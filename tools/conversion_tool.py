#!/bin/env python

from nuscenes.nuscenes import NuScenes
import os
import numpy as np

nusc = NuScenes(version='v1.0-mini', dataroot='/home/chros/nuscenes-mini', verbose = True)

scene_no = input("Enter scene number: " )
scene = None
for s in nusc.scene:
    if s['name'] == 'scene-{0}'.format(scene_no):
        scene = s
        break
if scene is None:
    print("Scene number not found")
    exit(1)

scene_no = str(int(scene_no)) # Removing padding

if not os.path.exists("/home/chros/rosbags/{0}".format(scene_no)):
    os.mkdir("/home/chros/rosbags/{0}".format(scene_no))


from rosbags.typesys import get_types_from_msg, register_types
from rosbags.rosbag1 import Writer, Reader
from rosbags.serde import serialize_cdr, cdr_to_ros1
from rosbags.typesys.types import geometry_msgs__msg__Vector3 as Vector3
from rosbags.typesys.types import geometry_msgs__msg__Quaternion as Quaternion
from rosbags.typesys.types import std_msgs__msg__Header as Header
from rosbags.typesys.types import builtin_interfaces__msg__Time as Time

DETECTED_OBJECT_MSG = """
geometry_msgs/Vector3 pose
string token
uint32 id
bool new_detection
bool is_vehicle
geometry_msgs/Vector3 box_size
geometry_msgs/Quaternion box_orientation
"""

DETECTED_OBJECT_ARRAY_MSG="""
std_msgs/Header header
aipd_msgs/DetectedObject[] objects
"""

register_types(get_types_from_msg(DETECTED_OBJECT_MSG, 'aipd_msgs/DetectedObject'))
register_types(get_types_from_msg(DETECTED_OBJECT_ARRAY_MSG, 'aipd_msgs/DetectedObjectArray'))

from rosbags.typesys.types import aipd_msgs__msg__DetectedObject as DetectedObject
from rosbags.typesys.types import aipd_msgs__msg__DetectedObjectArray as DetectedObjectArray
from rosbags.typesys.types import sensor_msgs__msg__CameraInfo as CameraInfo
from rosbags.typesys.types import sensor_msgs__msg__RegionOfInterest as RegionOfInterest

with Writer('/home/chros/rosbags/{0}/{0}-annotations.bag'.format(scene_no)) as writer:
    annotation_topic = '/detected_objects'
    annotation_type = DetectedObjectArray.__msgtype__
    annotation_connection = writer.add_connection(annotation_topic, annotation_type, latching=True, msgdef=DETECTED_OBJECT_ARRAY_MSG)

    camera_connections = []

    cam_front_topic = '/cam_front/camera_info'
    cam_front_type = CameraInfo.__msgtype__
    cam_front_connection = writer.add_connection(cam_front_topic, cam_front_type, latching=True)
    camera_connections.append((cam_front_type, cam_front_connection, 'CAM_FRONT'))

    cam_front_right_topic = '/cam_front_right/camera_info'
    cam_front_right_type = CameraInfo.__msgtype__
    cam_front_right_connection = writer.add_connection(cam_front_right_topic, cam_front_right_type, latching=True)
    camera_connections.append((cam_front_right_type, cam_front_right_connection, 'CAM_FRONT_RIGHT'))

    cam_front_left_topic = '/cam_front_left/camera_info'
    cam_front_left_type = CameraInfo.__msgtype__
    cam_front_left_connection = writer.add_connection(cam_front_left_topic, cam_front_left_type, latching=True)
    camera_connections.append((cam_front_left_type, cam_front_left_connection, 'CAM_FRONT_LEFT'))

    cam_back_topic = '/cam_back/camera_info'
    cam_back_type = CameraInfo.__msgtype__
    cam_back_connection = writer.add_connection(cam_back_topic, cam_back_type, latching=True)
    camera_connections.append((cam_back_type, cam_back_connection, 'CAM_BACK'))

    cam_back_right_topic = '/cam_back_right/camera_info'
    cam_back_right_type = CameraInfo.__msgtype__
    cam_back_right_connection = writer.add_connection(cam_back_right_topic, cam_back_right_type, latching=True)
    camera_connections.append((cam_back_right_type, cam_back_right_connection, 'CAM_BACK_RIGHT'))

    cam_back_left_topic = '/cam_back_left/camera_info'
    cam_back_left_type = CameraInfo.__msgtype__
    cam_back_left_connection = writer.add_connection(cam_back_left_topic, cam_back_left_type, latching=True)
    camera_connections.append((cam_back_left_type, cam_back_left_connection, 'CAM_BACK_LEFT'))

    sample = nusc.get('sample', scene['first_sample_token'])
    id_lookup_table = dict()
    id_counter = 0
    while sample['next'] != '':
        detected_objects = []
        timestamp = str(sample['timestamp'])
        secs = int(timestamp[:10])
        nsecs = int(timestamp[10:] + '000')
        time = Time(secs, nsecs)
        frame_id = "base_link"
        header = Header(time, frame_id)   
        for annotation_token in sample['anns']:
            sample_annotation = nusc.get('sample_annotation', annotation_token)
            pose_x = sample_annotation['translation'][0]
            pose_y = sample_annotation['translation'][1]
            pose_z = sample_annotation['translation'][2]
            pose = Vector3(pose_x, pose_y, pose_z)
            width = sample_annotation['size'][0]
            length = sample_annotation['size'][1]
            height = sample_annotation['size'][2]
            size = Vector3(length, width, height)
            w = sample_annotation['rotation'][0]
            x =sample_annotation['rotation'][1]
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
            detected_object = DetectedObject(pose, token, id, new_detection, is_vehicle, size, orientation)
            detected_objects.append(detected_object)
        message = DetectedObjectArray(header, detected_objects)
        writer.write(annotation_connection, int(str(secs) + str(nsecs)), cdr_to_ros1(serialize_cdr(message, annotation_type), annotation_type))
        for type, connection, modality in camera_connections:
            sample_data = nusc.get('sample_data', sample['data'][modality])
            sensor_object = nusc.get('calibrated_sensor', sample_data['calibrated_sensor_token'])
            timestamp = str(sample['timestamp'])
            secs = int(timestamp[:10])
            nsecs = int(timestamp[10:] + '000')
            time = Time(secs, nsecs)
            frame_id = modality.lower()
            header = Header(time, frame_id)
            calibration = np.asarray(sensor_object['camera_intrinsic']).flatten()
            camera_info = CameraInfo(header, int(sample_data['height']), int(sample_data['width']), 'NUSCENES', np.zeros(5), calibration, np.zeros(9), np.zeros(12), 0, 0, RegionOfInterest(0, 0, 0, 0, False))
            writer.write(connection, int(str(secs) + str(nsecs)), cdr_to_ros1(serialize_cdr(camera_info, type), type))
        sample = nusc.get('sample', sample['next'])