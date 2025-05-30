import bisect

def resample_pipeline(dt: float, episode_len: int):
    step_ns = int(dt * 1e9)

    def resample(data_dict: dict) -> dict:
        result = {}
        for topic, entries in data_dict.items():
            if not entries:
                result[topic] = []
                continue

            timestamps, msg_types, messages = zip(*entries)
            start_time = timestamps[0]

            resampled = []

            for i in range(episode_len):
                target_time = start_time + i * step_ns
                idx = bisect.bisect_left(timestamps, target_time)

                if idx == 0:
                    closest_idx = 0
                elif idx >= len(timestamps):
                    closest_idx = len(timestamps) - 1
                else:
                    before = timestamps[idx - 1]
                    after = timestamps[idx]
                    closest_idx = idx - 1 if abs(before - target_time) <= abs(after - target_time) else idx

                resampled.append((timestamps[closest_idx], msg_types[closest_idx], messages[closest_idx]))

            result[topic] = resampled

        return result

    return resample