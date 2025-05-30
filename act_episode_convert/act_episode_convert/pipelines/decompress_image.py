import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image

def decompress_compressed_images_pipeline():
    bridge = CvBridge()

    def decompress(data_dict: dict) -> dict:
        result = {}

        for topic, entries in data_dict.items():
            new_entries = []

            for timestamp, msg_type, msg in entries:
                if msg_type.__name__.endswith("CompressedImage"):
                    # decompress
                    try:
                        cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
                        raw_msg = bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
                        raw_msg.header = msg.header  # copy header
                        new_entries.append((timestamp, "sensor_msgs/msg/Image", raw_msg))
                    except Exception as e:
                        print(f"[WARN] Failed to decompress {topic} at {timestamp}: {e}")
                else:
                    new_entries.append((timestamp, msg_type, msg))

            result[topic] = new_entries

        return result

    return decompress