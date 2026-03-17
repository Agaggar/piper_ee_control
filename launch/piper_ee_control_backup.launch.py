from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
# from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os, subprocess
import yaml

import xacro


def load_yaml(package_name, file_path):
    """Load a YAML file from a ROS package."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    # ==========================================================================
    # Load Configurations
    # ==========================================================================
    piper_moveit_config_path = get_package_share_directory('piper_moveit_config')
    
    moveit_cpp_yaml_file_name = (get_package_share_directory("ee_velocity_controller") + "/config/moveit_cpp.yaml")
    # xacro
    # Component yaml files are grouped in separate namespaces
    robot_description_config = xacro.process_file(os.path.join(piper_moveit_config_path, 'config', 'piper.urdf.xacro'))
    robot_description = {"robot_description": robot_description_config.toxml()}
    # Semantic description (SRDF)
    robot_description_semantic = {'robot_description_semantic': load_file('piper_moveit_config', 'config/piper.srdf')}
    # Kinematics configuration
    kinematics_yaml = load_yaml('piper_moveit_config', 'config/kinematics.yaml')
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    # Planning configuration
    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        },
    }
    ompl_planning_yaml = load_yaml('piper_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)
    joint_limits_yaml = load_yaml('piper_moveit_config', 'config/joint_limits.yaml')

    # Servo configuration
    servo_yaml = os.path.join(piper_moveit_config_path, "config", "piper_servo.yaml")

    servo_node = Node(
        package="ee_velocity_controller",
        executable="ee_velocity_node",
        output="screen",
        parameters=[
            servo_yaml,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )
    
    # Controller configuration
    moveit_controllers = load_yaml('piper_moveit_config', 'config/moveit_controllers.yaml')
    old_moveit_cpp = Node(
            package="ee_velocity_controller",
            executable="ee_velocity_node",
            output="screen",
            parameters=[
                moveit_cpp_yaml_file_name,
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                ompl_planning_pipeline_config,
                moveit_controllers,
                joint_limits_yaml,
            ],
        )
    return LaunchDescription([
        servo_node
    ])