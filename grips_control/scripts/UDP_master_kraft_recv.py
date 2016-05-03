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

_struct_d10 = struct.Struct(">10d")
pi = 3.1415926535897931

class UDP_master_recv:
  def __init__(self):
      
    # Read parameters
    self.read_ip = self.read_parameter('~read_ip', '127.0.0.1')
    self.read_port = self.read_parameter('~read_port', '34900')
    self.publish_rate = self.read_parameter('~publish_rate', '1000.0')
    self.frame_id = self.read_parameter('~frame_id', 'world')
    self.joint_states_topic = self.read_parameter('~joint_states_topic', '/master_kraft/joint_states')

    # Publisher
    self.jointstate_topic_pub = rospy.Publisher(self.joint_states_topic, JointState)
    self.button1_pub = rospy.Publisher('/master_kraft/button_1', Float64)
    self.button2_pub = rospy.Publisher('/master_kraft/button_2', Float64)

    # Setup read socket
    self.read_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.read_socket.bind((self.read_ip, self.read_port))
    rospy.loginfo('UDP Socket listening on port [%d]' % (self.read_port))

    # Register rospy shutdown hook
    rospy.on_shutdown(self.shutdown_hook)

    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    joint_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joint_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    button1 = 0.0
    button2 = 0.0
    
    while True:
        data,addr=self.read_socket.recvfrom(1024)
        if data:        
            values = _struct_d10.unpack(data[0:80])
            
            joint_values[0] = (values[0]/0.0183)*pi/180
            joint_values[1] = ((values[1]-0.12)/0.0195)*pi/180
            joint_values[2] = ((values[1]+values[2]-3.0)/0.0186)*pi/180
            joint_values[3] = (values[4]/0.0196)*pi/180
            joint_values[4] = ((values[3]+0.55)/0.024)*pi/180
            joint_values[5] = (values[5]/0.021)*pi/180
            
            cmd_msg = JointState()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.header.frame_id = self.frame_id
            cmd_msg.name = list(joint_names)
            cmd_msg.position = list(joint_values)
            cmd_msg.velocity = list(joint_vel)
            
            if (values[7] > -3.0):
                button1 = 1.0
            else:
                button1 = 0.0
                
            if (abs(values[6]) > 8.0):
                button2 = 1.0
            else:
                button2 = 0.0

            try:
                self.jointstate_topic_pub.publish(cmd_msg)
                self.button1_pub.publish(Float64(button1))
                self.button2_pub.publish(Float64(button2))
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
