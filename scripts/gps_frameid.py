#!/usr/bin/env python3

from __future__ import print_function
import random
import numpy as np 
import rospy
from sensor_msgs.msg import NavSatFix

class alphasense_view():
  def __init__(self):

    gps_topic = '/fix'
    pub_gps_topic = '/fix/data'

    rospy.Subscriber(gps_topic, NavSatFix, self.gps_Callback)

    self.teju_gps_pub = rospy.Publisher(pub_gps_topic, NavSatFix, queue_size = 10)
    self.last_lat = None
    self.last_long = None

  def gps_Callback(self, msg):
    # rospy.loginfo("START of PROCESS")
    if self.last_lat is not None:
      lat_diff = msg.latitude - self.last_lat
      long_diff = msg.longitude - self.last_long
      self.last_lat = msg.latitude
      self.last_long = msg.longitude
      msg.latitude += lat_diff*random.random()
      msg.longitude += long_diff*random.random()
    msg.header.frame_id = "gps"
    self.teju_gps_pub.publish(msg)

if __name__ == '__main__':
  try:
    rospy.init_node('image_view_node')
    alpha = alphasense_view()    
    rospy.spin()

  except rospy.ROSInterruptException:
    rospy.loginfo("node terminated.")

