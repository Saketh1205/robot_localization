#!/usr/bin/env python3

from __future__ import print_function

import numpy as np 
import rospy
from nav_msgs.msg import Odometry


class alphasense_view():
  def __init__(self):

    odom_topic = '/odometry/filtered'
    pub_odom_topic = '/odometry/final'

    rospy.Subscriber(odom_topic, Odometry , self.odom_Callback)

    self.odom_pub = rospy.Publisher(pub_odom_topic, Odometry, queue_size = 10)

  def odom_Callback(self, msg):
    msg.pose.pose.position.x -= 1.2000
    # rospy.loginfo("START of PROCESS")
    self.odom_pub.publish(msg)

if __name__ == '__main__':
  try:
    rospy.init_node('final_odom')
    alpha = alphasense_view()    
    rospy.spin()

  except rospy.ROSInterruptException:
    rospy.loginfo("node terminated.")

