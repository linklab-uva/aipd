#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from aipd_msgs.msg import DetectedObjectArray, Ticket
import ros_numpy
import PIL.Image
import matplotlib.pyplot as plt
import numpy as np
from typing import Tuple
from nuscenes.utils.data_classes import Box
from pyquaternion import Quaternion
import tf2_ros, tf2_geometry_msgs

image_pub = None
current_boxes = DetectedObjectArray()
cam_front_calibration = None
tf_buffer = tf2_ros.Buffer()
i = 0
tickets = set()

def camera_bbox_visualizer():
    global image_pub, tf_buffer
    rospy.init_node('camera_bbox_publisher', anonymous=True)
    listener = tf2_ros.TransformListener(tf_buffer)
    image_pub = rospy.Publisher('cam_front/boxes', Image, queue_size=20)
    raw_image_sub = rospy.Subscriber('cam_front/raw', Image, camera_callback)
    image_calibration_sub = rospy.Subscriber('cam_front/camera_info', CameraInfo, calibration_callback)
    annotation_sub = rospy.Subscriber('detected_objects', DetectedObjectArray, object_callback)
    ticket_sub = rospy.Subscriber('tickets', Ticket, ticket_callback)
    rospy.spin()
    
# On callback, draw bounding box on image
def camera_callback(msg : Image):
    global image_pub, cam_front_calibration, i
    if cam_front_calibration is None:
        return
    image_arr = ros_numpy.numpify(msg)
    image = PIL.Image.fromarray(image_arr)
    _, ax = plt.subplots(1, 1, figsize=(9,16))
    ax.imshow(image)
    for box in current_boxes.objects:
        if not box.is_vehicle or box.is_moving:
            continue
        object_pose = Pose()
        object_pose.position = box.pose
        object_pose.orientation = box.box_orientation
        transformed_pose = transform_coordinates(object_pose, msg.header.stamp)
        if box.id in tickets:
            c = np.array((255, 0, 0)) / 255.0
        else:
            c = np.array((0, 255, 0)) / 255.0
        nuscenes_box = Box(
            [transformed_pose.position.x, transformed_pose.position.y, transformed_pose.position.z],
            [box.box_size.y, box.box_size.x, box.box_size.z],
            Quaternion(transformed_pose.orientation.w, transformed_pose.orientation.x, transformed_pose.orientation.y, transformed_pose.orientation.z),
        )
        nuscenes_box.render(ax, view=cam_front_calibration, normalize=True, colors=(c, c, c))
        
    ax.set_xlim(0, msg.width)
    ax.set_ylim(msg.height, 0)
    ax.axis('off')
    ax.set_aspect('equal')
    print("Picture saved")
    plt.savefig('/home/chros/test{0}.png'.format(i), bbox_inches='tight', pad_inches=0, dpi=200)
    i += 1
        
def object_callback(msg : DetectedObjectArray):
    global current_boxes
    current_boxes = msg

def calibration_callback(msg : CameraInfo):
    global cam_front_calibration
    cam_front_calibration = np.reshape(msg.K, (3,3))

def transform_coordinates(pose : Pose, stamp):
    global tf_buffer

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = "map"
    pose_stamped.header.stamp = stamp

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, "cam_front", rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

def ticket_callback(ticket : Ticket):
    global tickets
    tickets.add(ticket.id)

if __name__ == '__main__':
    camera_bbox_visualizer()