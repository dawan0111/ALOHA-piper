from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        LoadComposableNodes(
            target_container='my_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='act_episode_server',
                    plugin='ACT::EpisodeRecordServer',
                    name='episode_record_server',
                    parameters=[{
                        'record_path': 'episode_data',
                        'image_topic_names': [
                            '/camera1/image_compressed',
                            '/camera2/image_compressed',
                            '/camera3/image_compressed',
                            '/camera4/image_compressed',
                        ]
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                )
            ]
        )
    ])