#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from aipd_msgs.msg import DetectedObjectArray
import ros_numpy
import PIL.Image
import matplotlib.pyplot as plt
import numpy as np
from typing import Tuple
from nuscenes.utils.data_classes import Box
from pyquaternion import Quaternion

image_pub = None
current_boxes = DetectedObjectArray()
cam_front_calibration = np.eye(3)

def camera_bbox_visualizer():
    global image_pub
    rospy.init_node('camera_bbox_publisher', anonymous=True)
    image_pub = rospy.Publisher('cam_front/boxes', Image, queue_size=20)
    raw_image_sub = rospy.Subscriber('cam_front/raw', Image, camera_callback)
    image_calibration_sub = rospy.Subscriber('cam_front/camera_info', CameraInfo, calibration_callback)
    annotation_sub = rospy.Subscriber('detected_objects', DetectedObjectArray, object_callback)
    rospy.spin()
    
# On callback, draw bounding box on image
def camera_callback(msg : Image):
    global image_pub
    image_arr = ros_numpy.numpify(msg)
    image = PIL.Image(image_arr)
    _, ax = plt.subplots(1, 1, figsize=(9,16))
    ax.imshow(image)
    for box in current_boxes.objects:
        c = np.array((0, 0, 255)) / 255.0
        nuscenes_box = Box(
            [box.pose.x, box.pose.y, box.pose.z],
            [box.box_size.x, box.box_size.y, box.box_size.z],
            Quaternion(box.box_orientation.w, box.box_orientation.x, box.box_orientation.y, box.box_orientation.z),
        )
        nuscenes_box.render(ax, view=cam_front_calibration, normalize=True, colors=(c, c, c))
        
    ax.set_xlim(0, msg.width)
    ax.set_ylim(msg.height, 0)
    plt.savefig('/home/chros/test.jpg')
        
def object_callback(msg : DetectedObjectArray):
    global current_boxes
    current_boxes = msg

def calibration_callback(msg : CameraInfo):
    global cam_front_calibration
    cam_front_calibration = np.reshape(msg.K, (3,3))

if __name__ == '__main__':
    camera_bbox_visualizer()