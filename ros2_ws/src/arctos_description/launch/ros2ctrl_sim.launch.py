from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

# So Gazebo can find the robot's mesh files:
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
share_parent = os.path.dirname(get_package_share_directory('arctos_description'))
set_gz_path = SetEnvironmentVariable(
    'GZ_SIM_RESOURCE_PATH',
    share_parent + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', ''))

controllers_yaml = PathJoinSubstitution(
    [FindPackageShare('arctos_description'), 'config', 'ros2_controllers.yaml'])

robot_description = ParameterValue(Command([
    FindExecutable(name='xacro'), ' ',
    PathJoinSubstitution([FindPackageShare('arctos_description'), 'urdf', 'arctos_urdf.xacro']),
    ' controllers_yaml:=', controllers_yaml,
]), value_type=str)

# Figure eight over the table, gripper pointing down (same values as the demo)
FIGURE_EIGHT_CENTER = [0.000, -0.209, 0.238]
GRIPPER_DOWN = [0.343, 0.000, 0.000, 0.939]

# So Gazebo can find the robot's mesh files:
description_share = get_package_share_directory('arctos_description')
set_gz_path = SetEnvironmentVariable(
    'GZ_SIM_RESOURCE_PATH',
    os.path.dirname(description_share) + os.pathsep
    + os.environ.get('GZ_SIM_RESOURCE_PATH', ''))

def generate_launch_description():
    control_share = get_package_share_directory('arctos_control')
    use_sim_time = {'use_sim_time': True}

    # One robot description for everything (Gazebo, MoveIt, Servo). The
    # mapping fills in the controllers_yaml argument of the main xacro
    moveit_config = (
        MoveItConfigsBuilder('arctos_scene', package_name='arctos_moveit_config')
        .robot_description(
            file_path=os.path.join(description_share, 'urdf', 'arctos_urdf.xacro'),
            mappings={'controllers_yaml': os.path.join(
                description_share, 'config', 'ros2_controllers.yaml')})
        .to_moveit_configs()
    )
    with open(os.path.join(control_share, 'config', 'servo.yaml')) as f:
        servo_params = {'moveit_servo': yaml.safe_load(f)}

    # Gazebo:
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': '-r empty.sdf'}.items())

    # Robot state publisher: URDF + joint states => link transforms
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[moveit_config.robot_description, use_sim_time])

    # Parameter bridge: so ROS 2's and Gazebo's clocks agree
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    # One-shot processes: spawn the arm, then the two controllers
    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'arctos'])
    
    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster'])
    
    arm = Node(package='controller_manager', executable='spawner',
               arguments=['arctos_arm_controller'])

    # MoveIt planner
    move_group = Node(
        package='moveit_ros_move_group', executable='move_group',
        parameters=[
            moveit_config.to_dict(),
            use_sim_time,
            {
                'publish_robot_description_semantic': True,
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
            },
        ],
        output='screen')

    # MoveIt Servo
    servo = Node(
        package='moveit_servo', executable='servo_node', name='servo_node',
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            use_sim_time,
        ],
        output='screen')

    # Carrot controller and figure-eight publisher
    carrot = Node(
        package='arctos_control', executable='carrot_on_a_stick_controller',
        parameters=[use_sim_time], output='screen')
    
    figure_eight = Node(
        package='arctos_control', executable='figure_eight_publisher',
        parameters=[use_sim_time, {
            'center': FIGURE_EIGHT_CENTER,
            'orientation': GRIPPER_DOWN,
            'half_width': 0.10,
            'max_speed': 0.04,
        }],
        output='screen')

    return LaunchDescription([
        set_gz_path, gz_sim, clock_bridge, rsp, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm])),
        RegisterEventHandler(OnProcessExit(
            target_action=arm, on_exit=[move_group, servo, carrot, figure_eight])),
        # RegisterEventHandler(OnProcessExit(
        #     target_action=arm, on_exit=[move_group, servo])),
    ])