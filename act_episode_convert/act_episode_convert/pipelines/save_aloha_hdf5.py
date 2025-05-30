import h5py
import numpy as np

def save_to_hdf5_pipeline(hdf5_path: str, topic_remap: dict = None, max_timesteps: int = None):
    
    def save(data_dict: dict) -> dict:
        topic_remap_ = topic_remap or {}

        with h5py.File(hdf5_path, 'w', rdcc_nbytes=1024**2*2) as root:
            root.attrs['sim'] = False
            obs = root.create_group('observations')
            image = obs.create_group('images')

            for topic, entries in data_dict.items():
                if not entries:
                    continue

                ds_name = topic_remap_.get(topic, topic.strip('/').replace('/', '_'))

                timestamps, msg_type_strs, msgs = zip(*entries)
                msg_type_str = msg_type_strs[0]

                if msg_type_str == "sensor_msgs/msg/Image":
                    h, w = msgs[0].height, msgs[0].width
                    array = np.zeros((len(msgs), h, w, 3), dtype=np.uint8)

                    for i, msg in enumerate(msgs):
                        array[i] = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, -1))

                    if max_timesteps:
                        array = array[:max_timesteps]

                    image.create_dataset(ds_name, data=array, chunks=(1, h, w, 3), dtype='uint8')
                else:
                    print(f"[WARN] Unsupported msg_type={msg_type_str} for topic={topic}")
                    continue

        return data_dict

    return save
