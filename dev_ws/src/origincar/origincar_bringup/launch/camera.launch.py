import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python import get_package_share_directory

def generate_launch_description():
    nv12_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('hobot_codec') + '/launch/hobot_codec_decode.launch.py'),
            launch_arguments={
                'codec_in_mode': 'ros',
                'codec_out_mode': 'shared_mem',
                'codec_sub_topic': '/image',
                'codec_pub_topic': '/hbmem_img'
            }.items()
    )

    return LaunchDescription([
        Node(
            package='hobot_usb_cam',
            executable='hobot_usb_cam',
            name='hobot_usb_cam',
            parameters=[
                {"camera_calibration_file_path": "/opt/tros/lib/hobot_usb_cam/config/usb_camera_calibration.yaml"},
                {"frame_id": "default_usb_cam"},
                {"framerate": 30},
                {'image_width': 640},     # 降低分辨率可进一步减少数据量
                {'image_height': 480},    # 降低分辨率可进一步减少数据量
                {"io_method": "mmap"},
                {"pixel_format": "mjpeg"},
                {"video_device": "/dev/video8"},
                {"zero_copy": False}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        Node(
            package='hobot_codec',
            executable='hobot_codec_republish',
            output='screen',
            parameters=[
                {"channel": 1},
                {"in_mode": "ros"},
                {"in_format": "jpeg"},
                {"out_mode": "ros"},
                {"out_format": "bgr8"},
                {"sub_topic": "/image"},
                {"pub_topic": "/image_raw"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        nv12_codec_node,
    ])