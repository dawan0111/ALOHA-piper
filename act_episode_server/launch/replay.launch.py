from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription 

import yaml
import os

def generate_launch_description():
    act_description_path = os.path.join(
        get_package_share_directory('act_description'),
        'launch',
        'display.piper.launch.py',
    )
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        description='Name of the rosbag folder to replay (relative to record_path)'
    )

    recoding_config_path = os.path.join(
        get_package_share_directory('act_bringup'),
        'config',
        'episode_record.yaml'
    )
    with open(recoding_config_path, 'r') as f:
        config = yaml.safe_load(f)

    record_path = config.get('record_path', '/tmp/record')
    bag_path = LaunchConfiguration('bag_path')
    full_bag_path = PathJoinSubstitution([record_path, bag_path])

    bag_play_cmd = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', full_bag_path],
        output='screen'
    )

    act_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(act_description_path)
    )

    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
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
        act_description_launch,
        bag_path_arg,
        bag_play_cmd,
        container
    ])
