from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import os

def generate_launch_description():
    with_rviz_arg = DeclareLaunchArgument(
        'with_rviz',
        default_value='false',
        description='Whether to launch RViz'
    )

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        description='Path to the rosbag2 folder to replay'
    )

    with_rviz = LaunchConfiguration('with_rviz')
    bag_path = LaunchConfiguration('bag_path')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('act_bringup'), 'rviz', 'rviz.rviz')],
        condition=IfCondition(with_rviz),
        output='screen'
    )

    bag_play_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_path,
        ],
        output='screen'
    )

    return LaunchDescription([
        with_rviz_arg,
        bag_path_arg,
        rviz_node,
        bag_play_cmd
    ])
