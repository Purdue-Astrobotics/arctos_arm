"""Launch the original carrot demo with its figure-eight publisher disabled."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('arctos_control'), 'launch', 'carrot_demo.launch.py'])),
            launch_arguments={'figure_eight': 'false'}.items()),
    ])
