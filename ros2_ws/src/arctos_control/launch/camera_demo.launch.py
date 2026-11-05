"""Run the Gazebo carrot controller with a physical monochrome camera."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('calibration_path', description='NPZ with K, dist, image_size'),
        DeclareLaunchArgument('camera', default_value=''),
        DeclareLaunchArgument('tag_id', default_value='0'),
        DeclareLaunchArgument('tag_size', default_value='0.06'),
        DeclareLaunchArgument('gz_args', default_value='-r empty.sdf'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('servo_distance', default_value='0.0',
                              description='0 uses planning only; 0.25 enables Servo handoff'),
    ]
    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('arctos_control'), 'launch', 'carrot_demo.launch.py'])),
        launch_arguments={
            'figure_eight': 'false', 'gz_args': LaunchConfiguration('gz_args'),
            'rviz': 'false', 'servo_distance': LaunchConfiguration('servo_distance'),
            'target_topic': '/camera/target_pose'}.items())
    camera = Node(
        package='arctos_sensors', executable='gige_camera_publisher',
        parameters=[{'camera': ParameterValue(LaunchConfiguration('camera'), value_type=str),
                     'pixel_format': 'Mono8', 'frame': 'physical_camera_optical_frame',
                     'use_sim_time': True}], output='screen')
    bridge = Node(
        package='arctos_cv', executable='april_tag_pose_bridge',
        remappings=[('desired_pose', '/camera/target_pose')],
        parameters=[{
            'calibration_path': ParameterValue(LaunchConfiguration('calibration_path'), value_type=str),
            'tag_id': ParameterValue(LaunchConfiguration('tag_id'), value_type=int),
            'tag_size': ParameterValue(LaunchConfiguration('tag_size'), value_type=float),
            'use_sim_time': True}], output='screen')
    viewer = Node(
        package='rviz2', executable='rviz2', name='camera_demo_view',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('arctos_control'), 'config', 'camera_demo.rviz'])],
        parameters=[{'use_sim_time': True}], output='screen')
    return LaunchDescription(args + [GroupAction(actions=[demo], scoped=True), camera, bridge, viewer])
