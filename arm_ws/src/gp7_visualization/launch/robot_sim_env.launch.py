from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue  # Import this
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to URDF
    pkg_share = get_package_share_directory('gp7_visualization')
    default_model_path = os.path.join(pkg_share, 'urdf', 'gp7_robot.urdf.xacro')

    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'robot_description_file',
            default_value=default_model_path,
            description='Path to the URDF file',
        ),
    ]

    # Properly set robot description
    robot_description_content = ParameterValue(
        Command(['xacro', ' ', LaunchConfiguration('robot_description_file')]), value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # Nodes

    controller_manager_config = PathJoinSubstitution(
        [
            FindPackageShare('gp7_visualization'),
            'config',
            'gp7_robot_controllers.yaml',
        ]
    )
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controller_manager_config
        ],
        output="both",
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[{'use_sim_time': False}],
        arguments=['joint_state_broadcaster'],
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--param-file',
            controller_manager_config,
            ],
    )


    rviz_config_path = os.path.join(pkg_share, 'rviz', 'default.rviz')

    # Spawner nodes



    return LaunchDescription(declared_arguments + [
        #gripper_trajectory_controller_node,
        #trajectory_controller_node,
        node_robot_state_publisher,
        control_node,
    ])
