#!/usr/bin/env python3
#
# Copyright 2025 AIRO LABS., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import os
from pathlib import Path  # noqa: E402
import sys

dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)

from launch.launch_description import LaunchDescription
from camera_config import CameraConfig, USB_CAM_DIR  # noqa: E402

from launch import LaunchDescription  # noqa: E402
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

CAMERAS = []
CAMERAS.append(
    CameraConfig(
        name='camera1',
        param_path=Path(USB_CAM_DIR, 'config', 'params_1.yaml')
    ),
)
CAMERAS.append(
    CameraConfig(
        name='camera2',
        param_path=Path(USB_CAM_DIR, 'config', 'params_2.yaml')
    ),
)
CAMERAS.append(
    CameraConfig(
        name='camera3',
        param_path=Path(USB_CAM_DIR, 'config', 'params_3.yaml')
    ),
)
CAMERAS.append(
    CameraConfig(
        name='camera4',
        param_path=Path(USB_CAM_DIR, 'config', 'params_4.yaml')
    ),
)

def generate_launch_description() -> LaunchDescription:
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='usb_cam',
                    plugin='usb_cam::UsbCamNode',
                    name=camera.name,
                    namespace=camera.namespace,
                    remappings=camera.remappings,
                    parameters=[camera.param_path]
                ) for camera in CAMERAS
            ] + [
                ComposableNode(
                    package='act_episode_server',
                    plugin='ACT::EpisodeRecordServer',
                    name='episode_record_server',
                    parameters=[{
                        'record_path': 'episode_data',
                        'topic_names': [
                            '/camera1/image_compressed',
                            '/camera2/image_compressed',
                            '/camera3/image_compressed',
                            '/camera4/image_compressed',
                        ]
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='act_episode_server',
                    plugin='ACT::EpisodeReplayServer',
                    name='episode_replay_server',
                    parameters=[{
                        'image_topic_names': [
                            '/record/camera1/image_compressed',
                            '/record/camera2/image_compressed',
                            '/record/camera3/image_compressed',
                            '/record/camera4/image_compressed',
                        ]
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                )
            ],
            output='screen',
    )


    return LaunchDescription([
        container,
    ])