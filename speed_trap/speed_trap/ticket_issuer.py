#!/usr/bin/env python3

from cmath import inf
from turtle import speed
import rospy
# from lgsvl_msgs.msg import DetectedRadarObjectArray
from aipd_msgs.msg import DetectedObjectArray
from aipd_msgs.msg import Ticket
from geometry_msgs.msg import Vector3
from datetime import datetime
import numpy as np
from std_msgs.msg import Int16, String

ticket_pub = None
num_objects_pub = None
ticket_description_pub = None
issued_tickets = set()
speed_limit = 40  # mph
ego_velocity = inf
instance_tracking_pose = dict()
instance_tracking_time = dict()

def ticket_issuer():
    global ticket_pub, num_objects_pub, ticket_description_pub
    rospy.init_node('ticket_issuer')
    ticket_pub  = rospy.Publisher("tickets", Ticket, queue_size=20)
    ego_velocity_sub = rospy.Subscriber("ego_velocity", Vector3, ego_velocity_callback)
    objects  = rospy.Subscriber("detected_objects", DetectedObjectArray, detected_object_callback)
    # Publishers for rviz panel
    num_objects_pub = rospy.Publisher("num_objects", Int16, queue_size=20)
    ticket_description_pub = rospy.Publisher("ticket_description", String, queue_size=20)
    rospy.spin()
    

def detected_object_callback(msg : DetectedObjectArray):
    global ticket_pub, issued_tickets, speed_limit, instance_tracking_pose, instance_tracking_time
    objects = msg.objects
    num_objects = Int16()
    num_objects.data = len(msg.objects)
    num_objects_pub.publish(num_objects)
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
        if (object_velocity > (speed_limit + 10)) and (object_velocity < ego_velocity):
            issued_tickets.add(object.id)
            ticket = Ticket()
            ticket.id = object.id
            ticket.velocity = object_velocity
            ticket.violation_time = msg.header.stamp
            ticket_pub.publish(ticket)
            ticket_description = String()
            ticket_description.data = "Ticket issued at", datetime.fromtimestamp(int(str(msg.header.stamp.secs))), "to vehicle", ticket.id, "for moving at", int(ticket.velocity), "mph in a", speed_limit, "mph zone."
            ticket_description_pub.publish(ticket_description)
            print(ticket_description.data)

def convert_mph(speed):
    return 2.2369 * speed

def ego_velocity_callback(msg : Vector3):
    global ego_velocity
    ego_velocity = int(convert_mph(np.linalg.norm([msg.x, msg.y, msg.z])))

if __name__ == '__main__':
    ticket_issuer()