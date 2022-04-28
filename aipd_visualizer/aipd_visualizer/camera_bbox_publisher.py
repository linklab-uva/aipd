#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from aipd_msgs.msg import DetectedObjectArray, Ticket
import numpy as np
from nuscenes.utils.data_classes import Box
from pyquaternion import Quaternion
import tf2_ros, tf2_geometry_msgs
from cv_bridge import CvBridge
from nuscenes.utils.data_classes import Box
from nuscenes.utils.geometry_utils import box_in_image, BoxVisibility
import cv2
import sys

image_pub = None
current_boxes = DetectedObjectArray()
cam_calibration = None
tf_buffer = tf2_ros.Buffer()
tickets = set()
bridge = None
topic = ""
no_boxes = True

def camera_bbox_visualizer(topic_name):
    global image_pub, tf_buffer, bridge, topic
    topic = topic_name
    rospy.init_node('camera_bbox_publisher', anonymous=True)
    listener = tf2_ros.TransformListener(tf_buffer)
    image_pub = rospy.Publisher('{0}/boxes'.format(topic), Image, queue_size=20)
    raw_image_sub = rospy.Subscriber('{0}/raw'.format(topic), Image, camera_callback)
    image_calibration_sub = rospy.Subscriber('{0}/camera_info'.format(topic), CameraInfo, calibration_callback)
    annotation_sub = rospy.Subscriber('detected_objects', DetectedObjectArray, object_callback)
    ticket_sub = rospy.Subscriber('tickets', Ticket, ticket_callback)
    bridge = CvBridge()
    rospy.spin()
    
# On callback, draw bounding box on image
def camera_callback(msg : Image):
    global image_pub, cam_calibration, bridge, no_boxes
    if cam_calibration is None:
        image_pub.publish(msg)
        return
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    for object in current_boxes.objects:
        if not object.is_vehicle or not object.is_moving:
            continue
        object_pose = Pose()
        object_pose.position = object.pose
        object_pose.orientation = object.box_orientation
        transformed_pose = transform_coordinates(object_pose, msg.header.stamp)
        if object.id in tickets:
            c = (((255, 0, 0),) * 3)
        else:
            c = (((0, 255, 0),) * 3)
        print(c)
        nuscenes_box = Box(
            [transformed_pose.position.x, transformed_pose.position.y, transformed_pose.position.z],
            [object.box_size.y, object.box_size.x, object.box_size.z],
            Quaternion(transformed_pose.orientation.w, transformed_pose.orientation.x, transformed_pose.orientation.y, transformed_pose.orientation.z),
        )
        if not box_in_image(nuscenes_box, cam_calibration, (msg.width, msg.height), vis_level=BoxVisibility.ANY):
            continue
        nuscenes_box.render_cv2(cv_image, view=cam_calibration, normalize=True, colors=c)
    no_boxes = True
    image_pub.publish(bridge.cv2_to_imgmsg(cv_image, encoding='passthrough'))

        
def object_callback(msg : DetectedObjectArray):
    global current_boxes, no_boxes
    no_boxes = False
    current_boxes = msg

def calibration_callback(msg : CameraInfo):
    global cam_calibration
    cam_calibration = np.reshape(msg.K, (3,3))

def transform_coordinates(pose : Pose, stamp):
    global tf_buffer, topic 

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = "map"
    pose_stamped.header.stamp = stamp

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, topic, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

def ticket_callback(ticket : Ticket):
    global tickets
    tickets.add(ticket.id)

if __name__ == '__main__':
    argv = rospy.myargv(argv=sys.argv)
    if len(argv) != 2:
        print("Usage: rosrun aipd_visualizer camera_bbox_publisher.py [topic_name]")
        exit(1)
    camera_bbox_visualizer(argv[1])