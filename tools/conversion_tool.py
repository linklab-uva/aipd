#!/bin/env python

from turtle import speed
from nuscenes.nuscenes import NuScenes
import os
import rosbag
from aipd_msgs.msg import DetectedObject, DetectedObjectArray
from std_msgs.msg import Header, Int16
from geometry_msgs.msg import Vector3, Quaternion
import cv2
from cv_bridge import CvBridge

def write_annotations(scene_no):
    if not os.path.exists("/media/chros/G-DRIVE/aipd_bags/{0}".format(scene_no)):
        os.mkdir("/media/chros/G-DRIVE/aipd_bags/{0}".format(scene_no))

    with rosbag.Bag('/media/chros/G-DRIVE/aipd_bags/{0}/{0}-annotations.bag'.format(scene_no), 'w') as writer:
        sample = nusc.get('sample', scene['first_sample_token'])
        id_lookup_table = dict()
        id_counter = 0
        log = nusc.get('log', scene['log_token'])
        map_name = log['location']
        if map_name == 'singapore-onenorth':
            map_filename = '/media/chros/G-DRIVE/nuscenes-data/maps/53992ee3023e5494b90c316c183be829.jpg'
        elif map_name == 'boston-seaport':
            map_filename = '/media/chros/G-DRIVE/nuscenes-data/maps/36092f0b03a857c6a3403e25b4b7aab3.jpg'
        elif map_name == 'singapore-queenstown':
            map_filename = '/media/chros/G-DRIVE/nuscenes-data/maps/93406b464a165eaba6d9de76ca09f5da.jpg'
        else:
            map_filename = '/media/chros/G-DRIVE/nuscenes-data/maps/37819e65e09e5547b8a3ceaefba56bb2.jpg'
        map_image = cv2.imread(map_filename, cv2.IMREAD_UNCHANGED)
        bridge = CvBridge()
        map_msg = bridge.cv2_to_imgmsg(map_image, encoding="passthrough")
        timestamp = str(sample['timestamp'])
        secs = int(timestamp[:10])
        nsecs = int(timestamp[10:] + '000')
        header = Header()
        header.stamp.secs = secs
        header.stamp.nsecs = nsecs
        writer.write('map_image', map_msg, header.stamp)
        while sample['next'] != '':
            detected_objects = []
            timestamp = str(sample['timestamp'])
            secs = int(timestamp[:10])
            nsecs = int(timestamp[10:] + '000')
            frame_id = "base_link"
            header = Header()
            header.stamp.secs = secs
            header.stamp.nsecs = nsecs
            header.frame_id = frame_id
            for annotation_token in sample['anns']:
                sample_annotation = nusc.get('sample_annotation', annotation_token)
                if int(sample_annotation['visibility_token']) < 4:
                    continue
                pose_x = sample_annotation['translation'][0]
                pose_y = sample_annotation['translation'][1]
                pose_z = sample_annotation['translation'][2]
                pose = Vector3(pose_x, pose_y, pose_z)
                width = sample_annotation['size'][0]
                length = sample_annotation['size'][1]
                height = sample_annotation['size'][2]
                size = Vector3(length, width, height)
                w = sample_annotation['rotation'][0]
                x = sample_annotation['rotation'][1]
                y = sample_annotation['rotation'][2]
                z = sample_annotation['rotation'][3]
                orientation = Quaternion(x, y, z, w)
                instance = nusc.get('instance', sample_annotation['instance_token'])
                id = instance['token']
                if instance['token'] in id_lookup_table:
                    new_detection = False
                else:
                    id_lookup_table[instance['token']] = id_counter
                    id_counter += 1
                    new_detection = True
                token = instance['token']
                id = id_lookup_table[token]
                if sample_annotation['category_name'].split('.')[0] == 'vehicle':
                    is_vehicle = True
                else:
                    is_vehicle = False
                is_moving = True
                for attribute_token in sample_annotation['attribute_tokens']:
                    attribute = nusc.get('attribute', attribute_token)
                    if attribute['name'] == 'vehicle.parked':
                        is_moving = False
                detected_object = DetectedObject(pose, token, id, new_detection, is_vehicle, is_moving, size, orientation)
                detected_objects.append(detected_object)
            message = DetectedObjectArray(header, detected_objects)
            writer.write('detected_objects', message, header.stamp)
            sample = nusc.get('sample', sample['next'])


nusc = NuScenes(version='v1.0-trainval', dataroot='/media/chros/G-DRIVE/nuscenes-data', verbose = True)
scene_no = input("Enter scene number (q to quit): " )
while scene_no != 'q':
    scene = None
    for s in nusc.scene:
        if s['name'] == 'scene-{0}'.format(scene_no):
            scene = s
            break
    if scene is None:
        print("Scene number not found")
    else:
        scene_no = str(int(scene_no)) # Removing padding
        write_annotations(scene_no)
    scene_no = input("Enter scene number (q to quit): " )