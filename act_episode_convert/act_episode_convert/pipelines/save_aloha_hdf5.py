import h5py
import numpy as np

def save_to_hdf5_pipeline(hdf5_path: str, topic_remap: dict = None, max_timesteps: int = None):
    def save(data_dict: dict) -> dict:
        topic_remap_ = topic_remap or {}

        with h5py.File(hdf5_path, 'w', rdcc_nbytes=1024**2*2) as root:
            root.attrs['sim'] = False
            obs = root.create_group('observations')
            image = obs.create_group('images')

            for topic in data_dict:
                if data_dict[topic]:
                    msg_type_str = data_dict[topic][0][1].__name__

                    if msg_type_str not in ["Image", "CompressedImage"]:
                        continue
                    
                    ds_name = topic_remap_.get(topic, topic.strip('/').replace('/', '_'))
                    timestamps, _, msgs = zip(*data_dict[topic])
                    h, w = msgs[0].height, msgs[0].width
                    array = np.zeros((len(msgs), h, w, 3), dtype=np.uint8)
                    for i, msg in enumerate(msgs):
                        array[i] = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, -1))
                    if max_timesteps:
                        array = array[:max_timesteps]
                    image.create_dataset(ds_name, data=array, chunks=(1, h, w, 3), dtype='uint8')

            def get_joint_array(topic: str, key: str) -> np.ndarray:
                entries = data_dict.get(topic, [])
                return np.array([getattr(msg, key) for _, _, msg in entries], dtype=np.float64)

            left_pos = get_joint_array('/left_arm/joint_states', 'position')
            right_pos = get_joint_array('/right_arm/joint_states', 'position')
            left_vel = get_joint_array('/left_arm/joint_states', 'velocity')
            right_vel = get_joint_array('/right_arm/joint_states', 'velocity')
            left_eff = get_joint_array('/left_arm/joint_states', 'effort')
            right_eff = get_joint_array('/right_arm/joint_states', 'effort')

            # 각 time step 별로 concat → (T, 14)
            qpos = np.concatenate([left_pos, right_pos], axis=1)
            qvel = np.concatenate([left_vel, right_vel], axis=1)
            effort = np.concatenate([left_eff, right_eff], axis=1)

            # === 3. action ===
            left_cmd = get_joint_array('/left_arm/joint_ctrl', 'position')
            right_cmd = get_joint_array('/right_arm/joint_ctrl', 'position')
            action = np.concatenate([left_cmd, right_cmd], axis=1)

            # === 4. max_timesteps 적용 ===
            if max_timesteps:
                qpos = qpos[:max_timesteps]
                qvel = qvel[:max_timesteps]
                effort = effort[:max_timesteps]
                action = action[:max_timesteps]

            obs.create_dataset('qpos', data=qpos, dtype='float64')
            obs.create_dataset('qvel', data=qvel, dtype='float64')
            obs.create_dataset('effort', data=effort, dtype='float64')
            obs.create_dataset('action', data=action, dtype='float64')

            root.close()

        return data_dict
    return save
