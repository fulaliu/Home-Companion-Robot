from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sample_home_robot',
            executable='camera0_ws_node',
            name='camera0_ws',
            parameters=[{
                'input_topic': '/cam0_stream1',
                'host': '0.0.0.0',
                'port': 8765,
                'jpeg_quality': 80,
                'max_fps': 15
            }],
            output='screen'
        ),
        Node(
            package='sample_home_robot',
            executable='camera1_ws_node',
            name='camera1_ws',
            parameters=[{
                'input_topic': '/image_raw',
                'host': '0.0.0.0',
                'port': 8766,
                'jpeg_quality': 80,
                'max_fps': 15
            }],
            output='screen'
        ),
        Node(
            package='sample_home_robot',
            executable='cmd_ws_node',
            name='cmd_ws',
            parameters=[{
                'host': '0.0.0.0',
                'port': 8888,
                'cmd_topic': '/cmd_vel'
            }],
            output='screen'
        ),
    ])
