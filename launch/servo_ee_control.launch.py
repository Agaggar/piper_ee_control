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
from launch.actions import TimerAction, OpaqueFunction

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
    
def generate_launch_args():
    gripper_arg = DeclareLaunchArgument(
        'use_gripper',
        default_value='false',
        description='Whether to include gripper in the MoveIt configuration'
    )
    launch_args = [
        gripper_arg,
    ]
    return launch_args

def launch_setup(context):
    # ==========================================================================
    # Load Configurations
    # ==========================================================================
    piper_moveit_config_path = get_package_share_directory('piper_moveit_config')
    # if use_gripper is false, load the no_gripper version of the xacro and configs
    use_gripper = LaunchConfiguration('use_gripper').perform(context)
    if use_gripper.lower() == 'false':
        urdf_file = os.path.join(piper_moveit_config_path, 'config/no_gripper', 'piper.urdf.xacro')
        srdf_file = load_file('piper_moveit_config', 'config/no_gripper/piper.srdf')
        initial_positions_file = os.path.join(piper_moveit_config_path, "config/no_gripper", "initial_positions.yaml")
        # Controller configuration
        controllers_yaml_path = os.path.join(piper_moveit_config_path, "config/no_gripper", "ros2_controllers.yaml")
        # Kinematics configuration
        kinematics_yaml = load_yaml('piper_moveit_config', 'config/no_gripper/kinematics.yaml')
    else:
        urdf_file = os.path.join(piper_moveit_config_path, 'config', 'piper.urdf.xacro')
        srdf_file = load_file('piper_moveit_config', 'config/piper.srdf')
        initial_positions_file = os.path.join(piper_moveit_config_path, "config", "initial_positions.yaml")
        # Controller configuration
        controllers_yaml_path = os.path.join(piper_moveit_config_path, "config", "ros2_controllers.yaml")
        # Kinematics configuration
        kinematics_yaml = load_yaml('piper_moveit_config', 'config/kinematics.yaml')
    
    robot_description_config = xacro.process_file(
        urdf_file,
        mappings={"initial_positions_file": initial_positions_file})
    robot_description = {"robot_description": robot_description_config.toxml()}
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    # Semantic description (SRDF)
    robot_description_semantic = {'robot_description_semantic': srdf_file}

    rviz_config_file = os.path.join(get_package_share_directory('ee_velocity_controller'), 'config', 'robot_only.rviz')
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
            # planning_scene_monitor
        ]
    )

    # Controller configuration
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml_path],
        # remappings=[
        #     ("robot_description", "/piper/robot_description"),
        # ],
        output="screen",
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        # namespace='piper'
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        # output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    servo_params = (
        ParameterBuilder("ee_velocity_controller")
        .yaml(
            parameter_namespace="moveit_servo",
            file_path="config/piper_simulated_servo_params.yaml",
        )
        .to_dict()
    )
    if use_gripper.lower() == 'false':
        servo_params['moveit_servo']['ee_frame_name'] = 'link6'
    servo_demo_node = TimerAction(
            period=1.0,
            actions=[
                Node(
                    package="ee_velocity_controller",
                    executable="servo_demo_node",
                    output="screen",
                    parameters=[
                        servo_params,
                        robot_description,
                        robot_description_kinematics,
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
    # run keyboard rel_move in a separate terminal

    custom_js_broadcaster = Node(
        package="ee_velocity_controller",
        executable="custom_js_broadcaster",
        output="screen",
        parameters=[
            {
                'use_gripper': LaunchConfiguration('use_gripper'),
            }
        ],
        remappings=[
            ('~/arm_velocity_commands', '/arm_velocity_controller/commands'),
            ('~/arm_position_commands', '/arm_controller/joint_trajectory'),
        ],
    )

    return [
        robot_state_publisher_node,
        ## IMPORTANT! OVERRIDE JOINT STATE BROADCASTER. Solves random joint states order issue: https://github.com/ros-controls/ros2_controllers/issues/159
        custom_js_broadcaster,
        static_tf,
        servo_demo_node,
        velocity_relmove,
        rviz_node,
    ]

def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    launch_args = generate_launch_args()
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld