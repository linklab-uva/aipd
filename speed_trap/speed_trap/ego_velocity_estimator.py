#!/usr/bin/env python3

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
from datetime import datetime

previous_timestamp = None
previous_pose  = Point()
is_initialized = False
velocity_publisher = None

def ego_velocity_estimator():
    global velocity_publisher
    rospy.init_node('ego_velocity_estimator')
    velocity_publisher  = rospy.Publisher("ego_velocity", Int16, queue_size=20)
    ego_odom  = rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.spin()
    

def odom_callback(msg : Odometry):
    global previous_timestamp, previous_pose, is_initialized, velocity_publisher
    if not is_initialized:
        # print(int(str(msg.header.stamp.secs) + str(msg.header.stamp.nsecs)))
        previous_timestamp = datetime.fromtimestamp(int(str(msg.header.stamp.secs) + str(msg.header.stamp.nsecs)) / 1e9)
        previous_pose = msg.pose.pose.position
        is_initialized = True
        return
    current_time = datetime.fromtimestamp(int(str(msg.header.stamp.secs) + str(msg.header.stamp.nsecs)) / 1e9)
    time_difference = (current_time - previous_timestamp).total_seconds()
    # print(time_difference)
    ego_velocity_x = (msg.pose.pose.position.x - previous_pose.x) / time_difference
    ego_velocity_y = (msg.pose.pose.position.y - previous_pose.y) / time_difference
    ego_velocity_z = (msg.pose.pose.position.z - previous_pose.z) / time_difference
    ego_velocity = Int16()
    ego_velocity.data = int(convert_mph(np.linalg.norm([ego_velocity_x, ego_velocity_y, ego_velocity_z])))
    previous_timestamp = datetime.fromtimestamp(int(str(msg.header.stamp.secs) + str(msg.header.stamp.nsecs)) / 1e9)
    previous_pose = msg.pose.pose.position
    velocity_publisher.publish(ego_velocity)

def convert_mph(speed):
    return 2.2369 * speed

if __name__ == '__main__':
    ego_velocity_estimator()