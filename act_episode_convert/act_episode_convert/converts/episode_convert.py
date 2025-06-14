import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from collections import defaultdict
from act_episode_convert.pipelines import resample_pipeline, decompress_compressed_images_pipeline, save_to_hdf5_pipeline

class Compose:
    def __init__(self, transforms):
        self.transforms = transforms

    def __call__(self, x):
        for t in self.transforms:
            x = t(x)
        return x

class EpisodeConvert:
    def __init__(self, convert_pipeline: Compose):
        self.convert_pipeline = convert_pipeline
        pass

    def run(self, bagfile_path):
        data = self.__bagfile_to_dict(bagfile_path)
        self.__print_topic_diagnostics(data)

        return self.convert_pipeline(data)

    def __print_topic_diagnostics(self, data_dict):
        print("Topic Frequency & Count Diagnostic")
        print("-" * 60)

        for topic, entries in data_dict.items():
            timestamps = [ts for ts, _, _ in entries]
            n = len(timestamps)

            if n < 2:
                print(f"{topic:<40}  [insufficient data: {n} messages]")
                continue

            dts = np.diff(timestamps) / 1e9  # Convert ns to sec
            freq_mean = 1 / np.mean(dts)
            freq_std = np.std(1 / dts)

            print(f"{topic:<40}  {n:>5} msgs  |  {freq_mean:6.2f} Hz ± {freq_std:4.2f}")

        print("-" * 60)

    def __bagfile_to_dict(self, bagfile_path):
        storage_options = StorageOptions(uri=bagfile_path, storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        topic_type_map = {
            t.name: t.type
            for t in reader.get_all_topics_and_types()
        }

        topic_msg_map = defaultdict(list)

        while reader.has_next():
            topic_name, serialized_msg, timestamp = reader.read_next()
            msg_type = get_message(topic_type_map[topic_name])
            msg = deserialize_message(serialized_msg, msg_type)
            topic_msg_map[topic_name].append((timestamp, msg_type, msg))  # timestamp in ns

        return dict(topic_msg_map)
    
if __name__ == "__main__":
    compose = Compose([
        resample_pipeline(0.02, 100),
        decompress_compressed_images_pipeline(),
        save_to_hdf5_pipeline("/home/airo/tmp.hdf5")
    ])
    convert = EpisodeConvert(compose)
    data = convert.run("/home/airo/ros2_ws/episode_data")
    data_keys = list(data.keys())
    print("data:", len(data[data_keys[0]]))