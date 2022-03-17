#!/usr/bin/env python3

from turtle import speed
import rospy
# from lgsvl_msgs.msg import DetectedRadarObjectArray
from aipd_msgs.msg import DetectedObjectArray
from aipd_msgs.msg import Ticket
from geometry_msgs.msg import Vector3
from datetime import datetime
import numpy as np

ticket_pub = None
issued_tickets = set()
speed_limit = 40  # mph
ego_velocity = Vector3()
instance_tracking_pose = dict()
instance_tracking_time = dict()

def ticket_issuer():
    global ticket_pub
    rospy.init_node('ticket_issuer')
    ticket_pub  = rospy.Publisher("tickets", Ticket, queue_size=20)
    ego_velocity_sub = rospy.Subscriber("ego_velocity", Vector3, ego_velocity_callback)
    objects  = rospy.Subscriber("detected_objects", DetectedObjectArray, detected_object_callback)
    rospy.spin()
    

def detected_object_callback(msg : DetectedObjectArray):
    global ticket_pub, issued_tickets, speed_limit, instance_tracking_pose, instance_tracking_time
    objects = msg.objects
    for object in objects:
        if not object.is_vehicle or object.id in issued_tickets:
            continue
        if object.id not in instance_tracking_pose:
            instance_tracking_pose[object.id] = object.pose
            instance_tracking_time[object.id] = datetime.fromtimestamp(int(str(msg.header.stamp.secs) + str(msg.header.stamp.nsecs)) / 1e9)
            continue
        object_velocity_x = (object.pose.x - instance_tracking_pose[object.id].x) / (datetime.fromtimestamp(int(str(msg.header.stamp.secs) + str(msg.header.stamp.nsecs)) / 1e9) - instance_tracking_time[object.id]).total_seconds()
        object_velocity_y = (object.pose.y - instance_tracking_pose[object.id].y) / (datetime.fromtimestamp(int(str(msg.header.stamp.secs) + str(msg.header.stamp.nsecs)) / 1e9) - instance_tracking_time[object.id]).total_seconds()
        object_velocity_z = (object.pose.z - instance_tracking_pose[object.id].z) / (datetime.fromtimestamp(int(str(msg.header.stamp.secs) + str(msg.header.stamp.nsecs)) / 1e9) - instance_tracking_time[object.id]).total_seconds()
        object_velocity =  int(convert_mph(np.linalg.norm([object_velocity_x, object_velocity_y, object_velocity_z])))
        if (object_velocity > speed_limit):
            issued_tickets.add(object.id)
            ticket = Ticket()
            ticket.id = object.id
            ticket.velocity = object_velocity
            ticket.violation_time = msg.header.stamp
            ticket_pub.publish(ticket)
            print("Ticket issued at", datetime.fromtimestamp(int(str(msg.header.stamp.secs))), "to vehicle", ticket.id, "for moving at", int(ticket.velocity), "mph in a", speed_limit, "mph zone." )

def convert_mph(speed):
    return 2.2369 * speed

def ego_velocity_callback(msg : Vector3):
    global ego_velocity
    ego_velocity = msg

if __name__ == '__main__':
    ticket_issuer()