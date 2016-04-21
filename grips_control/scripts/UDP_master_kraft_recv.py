#! /usr/bin/env python
import rospy, time, math, os
import numpy as np
# Sockets
import socket, struct
# Messages
from StringIO import StringIO
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from sensor_msgs.msg import JointState

import roslib.message

_struct_d6 = struct.Struct("<6d")

class UDP_master_recv:
  def __init__(self):
      
    # Read parameters
    self.read_ip = self.read_parameter('~read_ip', '127.0.0.1')
    self.read_port = self.read_parameter('~read_port', '34900')
    self.publish_rate = self.read_parameter('~publish_rate', '1000.0')
    self.ref_frame = self.read_parameter('~ref_frame', 'world')

    # Publisher
    self.master_jointstate = '/master_kraft/joint_states'
    self.jointstate_topic_pub = rospy.Publisher(self.master_jointstate, JointState)


    # Setup read socket
    self.read_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.read_socket.bind((self.read_ip, self.read_port))
    rospy.loginfo('UDP Socket listening on port [%d]' % (self.read_port))

    # Register rospy shutdown hook
    rospy.on_shutdown(self.shutdown_hook)

    while True:
        data,addr=self.read_socket.recvfrom(1024)
        if data:        
            joint_values = _struct_d6.unpack(data[0:48])
            joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
            
            cmd_msg = JointState()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.header.frame_id = self.ref_frame
            cmd_msg.name = list(joint_names)
            cmd_msg.position = list(joint_values)

            try:
                self.jointstate_topic_pub.publish(cmd_msg)
            except:
                pass
  
                
  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)
        
  def shutdown_hook(self):
    # Do some cleaning depending on the app
    self.read_socket.close()
    pass

if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  interface = UDP_master_recv()
