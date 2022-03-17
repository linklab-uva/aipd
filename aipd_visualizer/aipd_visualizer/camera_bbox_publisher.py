#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from aipd_msgs.msg import DetectedObjectArray
import tf2_ros

image_pub = None
current_box = None
tf_buffer = tf2_ros.Buffer()

def camera_bbox_visualizer():
    global image_pub
    rospy.init_node('camera_bbox_publisher', anonymous=True)
    image_pub = rospy.Publisher('cam_front/boxes', Image, queue_size=20)
    listener = tf2_ros.TransformListener(tf_buffer)
    raw_image_sub = rospy.Subscriber('cam_front/raw', Image, camera_callback)
    annotation_sub = rospy.Subscriber('detected_objects', DetectedObjectArray, object_callback)
    rospy.spin()
    
# On callback, draw bounding box on image
def camera_callback(msg : Odometry):
    pass
    
def object_callback(msg : DetectedObjectArray):
    pass

if __name__ == '__main__':
    camera_bbox_visualizer()