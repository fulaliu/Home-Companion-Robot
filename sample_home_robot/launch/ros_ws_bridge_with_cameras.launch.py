# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 可选命名空间（方便在多相机或多实例时区分）
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for camera nodes'
    )
    namespace = LaunchConfiguration('namespace')

    # === QRB ROS Camera（Composable 组件方式） ===
    # 你给出的配置里要求用 PathJoinSubstitution；此处与 get_package_share_directory 结合使用
    camera_info_config_file_path = PathJoinSubstitution([
        get_package_share_directory('qrb_ros_camera'),
        'config', 'camera_info_imx577.yaml'
    ])

    qrb_camera_node = ComposableNode(
        package='qrb_ros_camera',
        namespace=namespace,
        plugin='qrb_ros::camera::CameraNode',   # 按你提供的插件类名
        name='camera_node',
        parameters=[{
            'camera_id': 0,
            'stream_size': 1,
            'stream_name': ['stream1'],
            'stream1': {
                'height': 480,
                'width':  640,
                'fps':    30,
            },
            'camera_info_path': camera_info_config_file_path,
            'dump': False,
            'dump_camera_info_': False,
        }],
        # 如果需要话题重映射，可在这里补充 remappings=[('xxx','yyy')]
    )

    qrb_camera_container = ComposableNodeContainer(
        name='qrb_camera_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container_mt',   # 多线程容器
        composable_node_descriptions=[qrb_camera_node],
        output='screen'
    )

    # === USB Camera（v4l2_camera 单独 Node） ===
    usb_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='usb_camera',
        parameters=[
            {'video_device': '/dev/video2'},
            {'image_size': [640, 480]},
            {'pixel_format': 'YUYV'},
            {'frame_rate': 30}
        ],
        output='screen'
    )

    # === 你原有的三个节点（WebSocket 推流+指令） ===
    # camera0_ws_node：假设将来接入 qrb camera 的 stream1 话题（你可以在此处改为真实的话题）
    camera0_ws = Node(
        package='sample_home_robot',
        executable='camera0_ws_node',
        name='camera0_ws',
        parameters=[{
            'input_topic': '/cam0_stream1',   # 如果 QRB 相机实际输出不是这个，请按需修改/重映射
            'host': '0.0.0.0',
            'port': 8765,
            'jpeg_quality': 80,
            'max_fps': 15
        }],
        output='screen'
    )

    # camera1_ws_node：这里我将输入改为 USB 摄像头的默认输出
    # v4l2_camera 的默认 raw 话题是 <node_name>/image_raw，因此为 /usb_camera/image_raw
    camera1_ws = Node(
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
    )

    cmd_ws = Node(
        package='sample_home_robot',
        executable='cmd_ws_node',
        name='cmd_ws',
        parameters=[{
            'host': '0.0.0.0',
            'port': 8888,
            'cmd_topic': '/cmd_vel'
        }],
        output='screen'
    )

    return LaunchDescription([
        namespace_arg,
        # composable 容器（QRB 相机）
        qrb_camera_container,
        # USB 摄像头节点
        usb_camera_node,
        # 你原有的三个节点
        camera0_ws,
        camera1_ws,
        cmd_ws
    ])