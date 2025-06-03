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
import yaml
from pathlib import Path  # noqa: E402
import sys

dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)

from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription 

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
    piper_description_path = os.path.join(
        get_package_share_directory('piper_description'),
        'launch',
        'piper_with_gripper',
        'display_xacro.launch.py'
    )
    recoding_cofig_path = os.path.join(
        get_package_share_directory('act_bringup'),
        'config',
        'episode_record.yaml'
    )
    with open(recoding_cofig_path, 'r') as f:
        config = yaml.safe_load(f)

    image_topics = config.get('topic_names', [])
    record_path = config.get('record_path', '/tmp/record')

    # Define launch parameters
    left_can_port_arg = DeclareLaunchArgument(
        'left_can_port',
        default_value='can_left',
        description='CAN port for the robot arm'
    )
    right_can_port_arg = DeclareLaunchArgument(
        'right_can_port',
        default_value='can_right',
        description='CAN port for the robot arm'
    )

    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='true',
        description='Enable robot arm automatically'
    )

    display_xacro_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(piper_description_path)
    )

    gripper_exist_arg = DeclareLaunchArgument(
        'gripper_exist',
        default_value='true',
        description='Gripper existence flag'
    )
    
    gripper_val_mutiple_arg = DeclareLaunchArgument(
        'gripper_val_mutiple',
        default_value='2',
        description='gripper'
    )

    # Define the robot arm node
    left_piper_ctrl_node = Node(
        package='piper',
        executable='piper_single_ctrl',
        name='left_piper_ctrl_single_node',
        output='screen',
        namespace='left_arm',
        parameters=[
            {'can_port': LaunchConfiguration('left_can_port')},
            {'auto_enable': LaunchConfiguration('auto_enable')},
            {'gripper_val_mutiple': LaunchConfiguration('gripper_val_mutiple')},
            {'gripper_exist': LaunchConfiguration('gripper_exist')}
        ],
    )

    right_piper_ctrl_node = Node(
        package='piper',
        executable='piper_single_ctrl',
        name='right_piper_ctrl_single_node',
        output='screen',
        namespace='right_arm',
        parameters=[
            {'can_port': LaunchConfiguration('right_can_port')},
            {'auto_enable': LaunchConfiguration('auto_enable')},
            {'gripper_val_mutiple': LaunchConfiguration('gripper_val_mutiple')},
            {'gripper_exist': LaunchConfiguration('gripper_exist')}
        ],
    )
    
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
                        'record_path': record_path,
                        'topic_names': image_topics
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='act_episode_server',
                    plugin='ACT::EpisodeReplayServer',
                    name='episode_replay_server',
                    parameters=[{
                        'image_topic_names': [
                            '/camera1/image_compressed',
                            '/camera2/image_compressed',
                            '/camera3/image_compressed',
                            '/camera4/image_compressed',
                        ]
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                )
            ],
            output='screen',
    )

    return LaunchDescription([
        left_can_port_arg,
        right_can_port_arg,
        auto_enable_arg,
        display_xacro_launch,
        gripper_exist_arg,
        gripper_val_mutiple_arg,
        left_piper_ctrl_node,
        right_piper_ctrl_node,
        container,
    ])