from nuscenes.nuscenes import NuScenes
import os

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
from rosbags.rosbag1 import Writer
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

with Writer('/home/chros/rosbags/{0}/{0}-annotations.bag'.format(scene_no)) as writer:
    annotation_topic = '/detected_objects'
    annotation_type = DetectedObjectArray.__msgtype__
    annotation_connection = writer.add_connection(annotation_topic, annotation_type, latching=True, msgdef=DETECTED_OBJECT_ARRAY_MSG)

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
        sample = nusc.get('sample', sample['next'])