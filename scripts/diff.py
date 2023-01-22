#!/usr/bin/env python3
import math
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class PoseDiffNode:
    def __init__(self):
        self.sub1 = rospy.Subscriber("gazebo/model_states", ModelStates, self.gazebo_callback)
        self.sub2 = rospy.Subscriber("odometry/final", Odometry, self.odom_callback)
        self.x_diff = None
        self.y_diff = None
        self.z_diff = None
        self.theta_diff = None

    def gazebo_callback(self, data):
        for i, name in enumerate(data.name):
            if name == "vehicle":  
                self.x1 = data.pose[i].position.x
                self.y1 = data.pose[i].position.y
                self.z1 = data.pose[i].position.z
                quaternion = (
                    data.pose[i].orientation.x,
                    data.pose[i].orientation.y,
                    data.pose[i].orientation.z,
                    data.pose[i].orientation.w)
                _, _, self.theta1 = euler_from_quaternion(quaternion)
                break

    def odom_callback(self, data):
        self.x2 = data.pose.pose.position.x
        self.y2 = data.pose.pose.position.y
        self.z2 = data.pose.pose.position.z
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        _, _, self.theta2 = euler_from_quaternion(quaternion)
        self.x_diff = self.x2 - self.x1
        self.y_diff = self.y2 - self.y1
        self.z_diff = self.z2 - self.z1
        self.euclid_dist = math.sqrt(self.x_diff**2 + self.y_diff**2)
        self.theta_diff = self.theta2 - self.theta1
        rospy.loginfo("Difference in pose: x: %f, y: %f, z: %f,ecldist: %f, theta: %f \n", self.x_diff, self.y_diff, self.z_diff, self.euclid_dist, self.theta_diff)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pose_diff_node')
    node = PoseDiffNode()
    node.run()





