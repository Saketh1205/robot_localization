#!/usr/bin/env python3

from __future__ import print_function

import numpy as np 
import rospy
from sensor_msgs.msg import Imu


class alphasense_view():
  def __init__(self):

    imu_topic = '/imu'
    pub_imu_topic = '/imu/data'

    rospy.Subscriber(imu_topic, Imu, self.imu_Callback)

    self.teju_imu_pub = rospy.Publisher(pub_imu_topic, Imu, queue_size = 10)

  def imu_Callback(self, msg):
    # rospy.loginfo("START of PROCESS")
    msg.header.frame_id = "imu_link"
    self.teju_imu_pub.publish(msg)

if __name__ == '__main__':
  try:
    rospy.init_node('image_view_node2')
    alpha = alphasense_view()    
    rospy.spin()

  except rospy.ROSInterruptException:
    rospy.loginfo("node terminated.")

