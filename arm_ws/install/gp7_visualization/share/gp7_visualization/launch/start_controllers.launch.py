from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    controllers_yaml = "/home/muneeb/my-robotic-arm/arm_ws/src/gp7_visualization/config/gp7_robot_controllers.yaml"

    return LaunchDescription([

        # Spawner for the JointTrajectoryController
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "real_arm_controller",
                "-c", "/real_robot/controller_manager",
                "-p", controllers_yaml,
            ],
            output="screen",
        ),

        # Spawner for the JointStateBroadcaster
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "real_state_controller",
                "-c", "/real_robot/controller_manager",
                "-p", controllers_yaml,
            ],
            output="screen",
        ),
    ])
