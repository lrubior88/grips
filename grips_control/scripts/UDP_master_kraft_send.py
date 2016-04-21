#!/usr/bin/env python

import rospy, os
import numpy as np
# PyKDL
import PyKDL
from tf_conversions import posemath
from pykdl_utils.kdl_kinematics import KDLKinematics, joint_list_to_kdl
# URDF
from urdf_parser_py.urdf import URDF
# Utils
from baxter_teleop.utils import read_parameter
# Messages
from baxter_core_msgs.msg import EndpointState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from StringIO import StringIO

_struct_d6 = struct.Struct("<6d")

def TwistMsgToKDL(msg_twist):
  vel = PyKDL.Vector(msg_twist.linear.x, msg_twist.linear.y, msg_twist.linear.z)
  rot = PyKDL.Vector(msg_twist.angular.x, msg_twist.angular.y, msg_twist.angular.z)
  return PyKDL.Twist(vel, rot)

class UDP_master_kraft_send(object):
  def __init__(self):
    # Read the controllers parameters
    self.publish_rate = read_parameter('~publish_rate', 1000)
    self.write_ip = self.read_parameter('~write_ip', '127.0.0.1')
    self.write_port = self.read_parameter('~write_port', '34900')
    self.frame_id = read_parameter('~frame_id', 'world')
    self.tip_link = read_parameter('~tip_link', 'eef')
    # Kinematics
    self.urdf = URDF.from_parameter_server(key='master_description')
    self.kinematics = KDLKinematics(self.urdf, self.frame_id, self.tip_link)
    # Get the joint names and limits
    self.joint_names = self.kinematics.get_joint_names()
    self.num_joints = len(self.joint_names)
    # Set-up publishers/subscribers
    rospy.Subscriber('/master_kraft/joint_states', JointState, self.joint_states_cb)
    rospy.Subscriber('/master_kraft/force_feedback', WrenchStamped, self.force_command_cb)
    while not rospy.is_shutdown():
        if (self.force_msg == None):
            rospy.sleep(0.01)
        else:
            break
    # Start torque controller timer
    self.joint_states = None
    
    # Set up write socket
    self.write_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
    self.torque_controller_timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.torque_controller_cb)
    # Shutdown hookup to clean-up before killing the script
    rospy.on_shutdown(self.shutdown)


  def joint_states_cb(self, msg):
    self.joint_states = msg
    
  def force_command_cb(self, msg):
    self.force_msg = msg

  def shutdown(self):
    self.torque_controller_timer.shutdown()
    # Stop the torque commands. It avoids unwanted movements after stoping this controller
    for i, name in enumerate(self.joint_names):
      self.torque_pub[name].publish(0.0)

  def torque_controller_cb(self, event):
    if rospy.is_shutdown() or None in [self.joint_states]:
      return
    
    ## Cartesian error to zero using a Jacobian transpose controller
    q, qd, eff = self.kinematics.extract_joint_state(self.joint_states)

    wrench = np.matrix(np.zeros(6)).T   
    wrench[0] = self.force_msg.wrench.force.x
    wrench[1] = self.force_msg.wrench.force.y 
    wrench[2] = self.force_msg.wrench.force.z 
    wrench[3] = self.force_msg.wrench.torque.x 
    wrench[4] = self.force_msg.wrench.torque.y 
    wrench[5] = self.force_msg.wrench.torque.z
    
    # Calculate the jacobian
    J = self.kinematics.jacobian(q)
    # Convert the force into a set of joint torques. tau = J^T * wrench
    tau = J.T * wrench
    
    buff = StringIO()
    buff.write(_struct_d6.pack(tau[0], tau[1], tau[2], tau[3], tau[4], tau[5]))
    # Send Buffer value
    try:
		self.write_socket.sendto(buff.getvalue(), (self.write_ip, self.write_port))
	except:
		rospy.logwarn('UDP_master_kraft_send: Connection problem')

if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  jtc = UDP_master_kraft_send()
  rospy.spin()
  rospy.loginfo('Shuting down [%s] node' % node_name)
