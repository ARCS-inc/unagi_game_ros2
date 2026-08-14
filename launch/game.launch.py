import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("unagi_game")
    params = os.path.join(share, "config", "game.yaml")
    show_viewer = LaunchConfiguration("show_viewer")

    return LaunchDescription([
        DeclareLaunchArgument("show_viewer", default_value="true"),
        Node(
            package="unagi_game",
            executable="physics_node",
            name="unagi_game_physics",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="unagi_game",
            executable="renderer_node",
            name="unagi_game_renderer",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="unagi_game",
            executable="viewer_node.py",
            name="unagi_game_viewer",
            output="screen",
            parameters=[params],
            condition=IfCondition(show_viewer),
        ),
    ])
