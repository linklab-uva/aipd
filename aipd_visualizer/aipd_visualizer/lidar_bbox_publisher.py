#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
from datetime import datetime
from aipd_msgs.msg import DetectedObjectArray
from geometry_msgs.msg import Pose, Point
import tf2_ros, tf2_geometry_msgs

marker_pub = None
ego_odom = Odometry()
tf_buffer = tf2_ros.Buffer()

def lidar_bbox_visualizer():
    global marker_pub, tf_buffer
    rospy.init_node('ego_velocity_estimator')
    marker_pub = rospy.Publisher('lidar_bboxes', MarkerArray, queue_size=20)
    listener = tf2_ros.TransformListener(tf_buffer)
    ego_odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)
    annotation_sub = rospy.Subscriber('detected_objects', DetectedObjectArray, object_callback)
    rospy.spin()
    

def odom_callback(msg : Odometry):
    global ego_odom
    ego_odom = msg

def object_callback(msg : DetectedObjectArray):
    global ego_odom
    marker_array : MarkerArray = MarkerArray()
    for object in msg.objects:
        if not object.is_vehicle:
            continue
        marker : Marker = Marker()
        marker.header = msg.header
        marker.ns = "boxes"
        object_pose = Pose()
        object_pose.position = object.pose
        object_pose.orientation = object.box_orientation
        marker_pose = transform_coordinates(object_pose, msg.header.stamp)
        marker.pose.position = marker_pose.position
        marker.pose.orientation = marker_pose.orientation
        marker.id = object.id
        marker.type = Marker.CUBE
        marker.scale = object.box_size
        marker.color.b = 1.0
        marker.color.a = 1.0
        # marker.points.append(marker_pose.position)
        marker_array.markers.append(marker)
    marker_pub.publish(marker_array)

    

def transform_coordinates(pose : Pose, stamp):
    global tf_buffer

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = "map"
    pose_stamped.header.stamp = stamp

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, "base_link", rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

if __name__ == '__main__':
    lidar_bbox_visualizer()