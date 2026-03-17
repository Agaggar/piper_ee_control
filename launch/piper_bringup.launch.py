from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command, LaunchConfiguration
# from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os, subprocess
import yaml
from launch_param_builder import ParameterBuilder

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
    moveit_arg = DeclareLaunchArgument(
        name='use_moveit',
        default_value='true',
        description='Enable MoveIt'
    )
    # ==========================================================================
    # Load Configurations
    # ==========================================================================
    piper_moveit_config_path = get_package_share_directory('piper_moveit_config')
    
    robot_description_config = xacro.process_file(
        os.path.join(piper_moveit_config_path, 'config', 'piper.urdf.xacro'),
        mappings={"initial_positions_file": os.path.join(piper_moveit_config_path, "config", "initial_positions.yaml")})
    robot_description = {"robot_description": robot_description_config.toxml()}
    kinematics_yaml = load_yaml('piper_moveit_config', 'config/kinematics.yaml')
    
    # Semantic description (SRDF)
    robot_description_semantic = {'robot_description_semantic': load_file('piper_moveit_config', 'config/piper.srdf')}
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

    # Controller configuration
    moveit_controllers = load_yaml('piper_moveit_config', 'config/moveit_controllers.yaml')
    controllers_yaml_path = os.path.join(piper_moveit_config_path, "config", "ros2_controllers.yaml")
    # control nodes
    controllers_yaml_path = os.path.join(piper_moveit_config_path, "config", "ros2_controllers.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_yaml_path
        ],
        output="screen",
    )
    load_controllers = []
    controllers = ["joint_state_broadcaster", "arm_controller", "gripper_controller"] #  "arm_velocity_controller"
    for controller in controllers:
        load_controllers += [
            TimerAction(
            period=1.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner.py",
                    arguments=[
                        controller,
                        "--controller-manager",
                        "/controller_manager"
                    ],
                )
            ]
        )
        ]
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    # A node to publish world -> base_link transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # ==========================================================================
    # MoveIt Move Group Node
    # ==========================================================================
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)
    
    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    
    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
            {'robot_description_planning': joint_limits_yaml},
            ompl_planning_pipeline_config,
            trajectory_execution,
            {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'},
            {'moveit_simple_controller_manager': moveit_controllers.get('moveit_simple_controller_manager', {})},
            planning_scene_monitor,
        ],
        condition=LaunchConfigurationEquals('use_moveit', 'true')
    )
    rviz_config_file = os.path.join(piper_moveit_config_path, 'rviz', 'moveit.rviz')
    # rviz_config_file = os.path.join(get_package_share_directory('ee_velocity_controller'), 'config', 'rviz_nomoveit.rviz')
    
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_scene_monitor
        ]
    )
    servo_params = (
        ParameterBuilder("ee_velocity_controller")
        .yaml(
            parameter_namespace="moveit_servo",
            file_path="config/piper_simulated_servo_params.yaml",
        )
        .to_dict()
    )
    servo_demo_node = TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="ee_velocity_controller",
                    executable="servo_demo_node",
                    output="screen",
                    parameters=[
                        servo_params,
                        robot_description,
                        robot_description_semantic,
                    ],
                )
            ]
        )
    velocity_relmove = Node(
        package="ee_velocity_controller",
        executable="velocity_pub",
        output="screen",
    )
    return LaunchDescription([
        moveit_arg,
        robot_state_publisher_node,
        ros2_control_node,
        *load_controllers,
        static_tf,
        move_group_node,
        rviz_node,
        servo_demo_node,
        velocity_relmove,
    ])
