import os

from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    default_model_launch = os.path.join(
        get_package_share_directory('piper_description'),
        'launch',
        'piper_with_gripper',
        'display_xacro.launch.py'
    )
    default_rviz_config = os.path.join(
        get_package_share_directory('act_description'),
        'rviz',
        'piper.rviz'
    )

    model_launch = IncludeLaunchDescription(
        default_model_launch
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=default_rviz_config,
        description='Path to RViz config file'
    )

    return LaunchDescription([
        rviz_config_arg,
        model_launch
    ])