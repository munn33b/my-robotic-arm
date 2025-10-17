import os
import launch
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
import xacro
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler

def generate_launch_description():
    ros_gz_bridge_config_file_path = 'config/ros_gz_bridge.yaml'

    package_name_gazebo = 'sample_description'
    pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
    default_robot_name = 'VBot_v1'
    pkg_name = FindPackageShare(package="sample_description").find("sample_description")

    robot_name = LaunchConfiguration('robot_name')

    default_ros_gz_bridge_config_file_path = os.path.join(pkg_name, ros_gz_bridge_config_file_path)

    # Declare the launch argument first
    declare_robot_name_cmd = DeclareLaunchArgument(
      name='robot_name',
      default_value=default_robot_name,
      description='The name for the robot')


    urdf_file_path = 'urdf/sample_humanoid_robot.urdf.xacro'
    default_urdf_model_path = os.path.join(pkg_share_gazebo, urdf_file_path)

    doc = xacro.parse(open(default_urdf_model_path))
    xacro.process_doc(doc)

    bras_robot_description_path = FindPackageShare("sample_description").find("sample_description")

    # Define the models directory inside the package
    models_path = os.path.join(bras_robot_description_path, "models")
    workspace_src_path = os.path.abspath(os.path.join(bras_robot_description_path, "../../../sample_description/share"))


    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = f"{models_path}:{workspace_src_path}"
    print (os.environ["IGN_GAZEBO_RESOURCE_PATH"])

    start_gazebo_ros_spawner_cmd = Node(
      package='ros_gz_sim',
      executable='create',
      arguments=[
        '-string', doc.toxml(),
        '-name', robot_name,
        '-allow_renaming', 'true',
        '-x', '0.663025',
        '-y', '-0.449473',
        '-z', '1.045598',
        '-R', '0',
        '-P', '0',
        '-Y', '-3.14',
        ],
      output='screen')

    robot_description_content = doc.toxml()

    robot_description = {'robot_description': robot_description_content}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    return launch.LaunchDescription([
        declare_robot_name_cmd,  # Ensure this is declared first
        start_gazebo_ros_spawner_cmd,
        node_robot_state_publisher
    ])
