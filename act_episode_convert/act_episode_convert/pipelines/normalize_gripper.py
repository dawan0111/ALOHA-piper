def normalize_gripper_pipeline():
    def normalize(data_dict: dict) -> dict:
        for topic, entries in data_dict.items():
            if not entries:
                continue

            msg_type = entries[0][1]
            # print(f"Processing topic '{topic}' with message type '{msg_type}'")
            if msg_type.__name__ == 'JointState':
                new_entries = []
                for ts, typ, msg in entries:
                    if len(msg.position) >= 7:
                        original = msg.position[6]
                        clipped = min(max(original, 0.0), 0.07)
                        normalized = clipped / 0.07
                        msg.position[6] = normalized

                    new_entries.append((ts, typ, msg))
                data_dict[topic] = new_entries
        return data_dict
    return normalize