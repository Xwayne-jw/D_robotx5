import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'avoid_angular_ratio',
            default_value='0.2',
            description='angular speed ratio for obstacle avoidance'),
        DeclareLaunchArgument(
            'avoid_linear_speed',
            default_value='0.1',
            description='linear speed for obstacle avoidance'),
        DeclareLaunchArgument(
            'follow_angular_ratio',
            default_value='-1.0',
            description='angular speed ratio for line following'),
        DeclareLaunchArgument(
            'follow_linear_speed',
            default_value='0.1',
            description='linear speed for line following'),
        DeclareLaunchArgument(
            'bottom_threshold',
            default_value='200',
            description='bottom threshold for line following'),
        DeclareLaunchArgument(
            'line_confidence_threshold',
            default_value='0.5',
            description='confidence threshold for line following'),
        DeclareLaunchArgument(
            'obstacle_confidence_threshold',
            default_value='0.5',
            description='confidence threshold for obstacle avoidance'),
        Node(
            package='racing_control',
            executable='racing_control',
            output='screen',
            parameters=[
                {"pub_control_topic": "/cmd_vel"},
                {"avoid_angular_ratio": LaunchConfiguration('avoid_angular_ratio')},
                {"avoid_linear_speed": LaunchConfiguration('avoid_linear_speed')},
                {"follow_angular_ratio": LaunchConfiguration('follow_angular_ratio')},
                {"follow_linear_speed": LaunchConfiguration('follow_linear_speed')},
                {"bottom_threshold": LaunchConfiguration('bottom_threshold')},
                {"line_confidence_threshold": LaunchConfiguration('line_confidence_threshold')},
                {"obstacle_confidence_threshold": LaunchConfiguration('obstacle_confidence_threshold')},
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        )
    ])
