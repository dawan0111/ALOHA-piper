import threading
import numpy as np
import cv2
from typing import Dict, Any
from sensor_msgs.msg import CompressedImage, JointState
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from act_environment.envs.observers._observer import ObserverInterface

class AlohaObserver(ObserverInterface):
    def __init__(self, node: Node):
        super().__init__(node)
        self._lock = threading.Lock()
        self._callback_group = ReentrantCallbackGroup()

        # 최신 데이터 저장
        self._images = {}  # topic_name → np.ndarray
        self._joint_state_left = None
        self._joint_state_right = None

        # Subscribe compressed images
        image_topics = [
            '/camera1/image_raw/compressed',
            '/camera2/image_raw/compressed',
            '/camera3/image_raw/compressed',
            '/camera4/image_raw/compressed',
        ]
        for topic in image_topics:
            node.create_subscription(
                CompressedImage,
                topic,
                lambda msg, topic=topic: self._image_callback(msg, topic),
                10,
                callback_group=self._callback_group
            )

        # Subscribe joint states
        node.create_subscription(
            JointState,
            '/left_arm/joint_states',
            self._joint_callback_left,
            10,
            callback_group=self._callback_group
        )
        node.create_subscription(
            JointState,
            '/right_arm/joint_states',
            self._joint_callback_right,
            10,
            callback_group=self._callback_group
        )

    def _image_callback(self, msg: CompressedImage, topic_name: str):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if image is not None:
                with self._lock:
                    self._images[topic_name] = image
        except Exception as e:
            self.node.get_logger().warn(f"[DecodeFail] {topic_name}: {e}")

    def _joint_callback_left(self, msg: JointState):
        with self._lock:
            self._joint_state_left = msg

    def _joint_callback_right(self, msg: JointState):
        with self._lock:
            self._joint_state_right = msg

    def get_observation(self) -> Dict[str, Any]:
        with self._lock:
            images_copy = {
                k.split('/')[-2]: v.copy()  # e.g., 'camera1'
                for k, v in self._images.items()
            }

            if self._joint_state_left is None or self._joint_state_right is None:
                qpos = np.zeros(14, dtype=np.float32)
                qvel = np.zeros(14, dtype=np.float32)
                effort = np.zeros(14, dtype=np.float32)
            else:
                qpos = np.concatenate([self._joint_state_left.position, self._joint_state_right.position])
                qvel = np.concatenate([self._joint_state_left.velocity, self._joint_state_right.velocity])
                effort = np.concatenate([self._joint_state_left.effort, self._joint_state_right.effort])

        return {
            "images": images_copy,
            "qpos": np.array(qpos, dtype=np.float32),
            "qvel": np.array(qvel, dtype=np.float32),
            "effort": np.array(effort, dtype=np.float32),
        }
