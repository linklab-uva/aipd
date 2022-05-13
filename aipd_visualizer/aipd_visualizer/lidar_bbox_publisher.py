#!/usr/bin/env python3

from turtle import speed
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
from datetime import datetime
from aipd_msgs.msg import DetectedObjectArray
from geometry_msgs.msg import Pose, Point
import tf2_ros, tf2_geometry_msgs
from aipd_msgs.msg import Ticket
from std_msgs.msg import Int16
import numpy as np

marker_pub = None
ego_odom = Odometry()
tf_buffer = tf2_ros.Buffer()
tickets = set()
last_annotation_time = None
speed_limit = None
last_annotation_set = dict()

def lidar_bbox_visualizer():
    global marker_pub, tf_buffer
    rospy.init_node('lidar_bbox_publisher')
    marker_pub = rospy.Publisher('lidar_bboxes', MarkerArray, queue_size=20)
    listener = tf2_ros.TransformListener(tf_buffer)
    ego_odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)
    annotation_sub = rospy.Subscriber('detected_objects', DetectedObjectArray, object_callback)
    ticket_sub = rospy.Subscriber('tickets', Ticket, ticket_callback)
    speed_limit_sub = rospy.Subscriber('speed_limit', Int16, speed_limit_callback)
    rospy.spin()
    

def odom_callback(msg : Odometry):
    global ego_odom
    ego_odom = msg

def object_callback(msg : DetectedObjectArray):
    global ego_odom, tickets, object_velocities, speed_limit, last_annotation_time
    if speed_limit is None:
        print("Speed limit not set")
        return
    marker_array : MarkerArray = MarkerArray()
    for object in msg.objects:
        if not object.is_vehicle or not object.is_moving:
            continue
        if object.new_detection:
            last_annotation_set[object.id] = object
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
        object_velocity = 0
        if last_annotation_time is not None:
            object_velocity_x = (object.pose.x - last_annotation_set[object.id].pose.x) / (datetime.fromtimestamp(int(str(msg.header.stamp.secs) + str(msg.header.stamp.nsecs)) / 1e9) - last_annotation_time).total_seconds()
            object_velocity_y = (object.pose.y - last_annotation_set[object.id].pose.y) / (datetime.fromtimestamp(int(str(msg.header.stamp.secs) + str(msg.header.stamp.nsecs)) / 1e9) - last_annotation_time).total_seconds()
            object_velocity_z = (object.pose.z - last_annotation_set[object.id].pose.z) / (datetime.fromtimestamp(int(str(msg.header.stamp.secs) + str(msg.header.stamp.nsecs)) / 1e9) - last_annotation_time).total_seconds()
            object_velocity =  int(convert_mph(np.linalg.norm([object_velocity_x, object_velocity_y, object_velocity_z])))
            last_annotation_set[object.id] = object
        if object.id in tickets:
            marker.color.a = 0.7
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif object_velocity > speed_limit:
            marker.color.a = 0.7
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.a = 0.7
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        marker.lifetime = rospy.Duration(0, 5e8)
        marker_array.markers.append(marker)
    last_annotation_time = datetime.fromtimestamp(msg.header.stamp.to_sec())
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

def ticket_callback(ticket : Ticket):
    global tickets
    tickets.add(ticket.id)

def speed_limit_callback(msg : Int16):
    global speed_limit
    speed_limit = msg.data

def convert_mph(speed):
    return 2.2369 * speed

if __name__ == '__main__':
    lidar_bbox_visualizer()