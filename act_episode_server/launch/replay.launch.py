from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
import yaml
import os

def generate_launch_description():
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

    return LaunchDescription([
        bag_path_arg,
        bag_play_cmd
    ])
