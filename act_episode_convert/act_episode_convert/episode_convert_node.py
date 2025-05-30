import rclpy

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from collections import defaultdict

def load_bag_as_topic_dict(bag_path: str) -> dict:
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_type_map = {
        t.name: t.type
        for t in reader.get_all_topics_and_types()
    }

    topic_msg_map = defaultdict(list)

    print(topic_msg_map)

    while reader.has_next():
        topic_name, serialized_msg, timestamp = reader.read_next()
        msg_type = get_message(topic_type_map[topic_name])
        msg = deserialize_message(serialized_msg, msg_type)
        topic_msg_map[topic_name].append(msg)

    return dict(topic_msg_map)

def main(args=None):
    result = load_bag_as_topic_dict("/home/airo/ros2_ws/episode_data")
    key = list(result.keys())
    print(type(result[key[3]][0]))
    # print(len(result.values()[0]))
    pass

if __name__ == "__main__":
    main()