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
    moveit_arg = DeclareLaunchArgument(
        name='use_moveit',
        default_value='true',
        description='Enable MoveIt'
    )
    jacobian_base_link_arg = DeclareLaunchArgument(
        name='jacobian_base_link',
        default_value='base_link',
        description='Base link for Jacobian chain'
    )
    jacobian_ee_link_arg = DeclareLaunchArgument(
        name='jacobian_ee_link',
        default_value='link6',
        description='End-effector link for Jacobian chain'
    )
    jacobian_joint_states_topic_arg = DeclareLaunchArgument(
        name='jacobian_joint_states_topic',
        default_value='/joint_states',
        description='Joint states topic consumed by get_jacobian_node'
    )
    control_rate_arg = DeclareLaunchArgument(
        name='control_rate_hz',
        default_value='30.0',
        description='Frequency for velocity ctrl (Hz)'
    )
    control_mode_arg = DeclareLaunchArgument(
        name='control_mode',
        default_value='position',
        description='Jacobian output mode: position or velocity'
    )
    launch_args = [
        gripper_arg,
        moveit_arg,
        jacobian_base_link_arg,
        jacobian_ee_link_arg,
        jacobian_joint_states_topic_arg,
        control_rate_arg,
        control_mode_arg,
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
        moveit_controllers = load_yaml('piper_moveit_config', 'config/no_gripper/moveit_controllers.yaml') # REAL controllers
        controllers_yaml_path = os.path.join(piper_moveit_config_path, "config/no_gripper", "ros2_controllers.yaml")
        # Kinematics configuration
        kinematics_yaml = load_yaml('piper_moveit_config', 'config/no_gripper/kinematics.yaml')
        joint_limits_yaml = load_yaml('piper_moveit_config', 'config/no_gripper/joint_limits.yaml')
        joint_limits_yaml_path = os.path.join(piper_moveit_config_path, "config/no_gripper", "joint_limits.yaml")
        # Planning configuration
        ompl_planning_yaml = load_yaml('piper_moveit_config', 'config/no_gripper/ompl_planning.yaml')
    else:
        urdf_file = os.path.join(piper_moveit_config_path, 'config', 'piper.urdf.xacro')
        srdf_file = load_file('piper_moveit_config', 'config/piper.srdf')
        initial_positions_file = os.path.join(piper_moveit_config_path, "config", "initial_positions.yaml")
        # Controller configuration
        moveit_controllers = load_yaml('piper_moveit_config', 'config/moveit_controllers.yaml') # REAL controllers
        controllers_yaml_path = os.path.join(piper_moveit_config_path, "config", "ros2_controllers.yaml")
        # Kinematics configuration
        kinematics_yaml = load_yaml('piper_moveit_config', 'config/kinematics.yaml')
        joint_limits_yaml = load_yaml('piper_moveit_config', 'config/joint_limits.yaml')
        joint_limits_yaml_path = os.path.join(piper_moveit_config_path, "config", "joint_limits.yaml")
        # Planning configuration
        ompl_planning_yaml = load_yaml('piper_moveit_config', 'config/ompl_planning.yaml')
    
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
    load_controllers = []
    control_mode = LaunchConfiguration('control_mode').perform(context).lower()
    controllers = ["joint_state_broadcaster"]
    # Design decision: spawn only one arm command controller to avoid mixed-command behavior.
    if control_mode == 'position':
        controllers += ["arm_controller"]
    else:
        controllers += ["arm_velocity_controller"]
    if use_gripper.lower() == 'true':
        controllers += ["gripper_controller"]
    for controller in controllers:
        load_controllers += [
            TimerAction(
            period=0.5,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner.py",
                    arguments=[
                        controller,
                        "--controller-manager", "/controller_manager"
                    ],
                )
            ]
        )
        ]

    # Joint State Publisher Node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{"publish_rate": 200}]
    )
    # Joint State Publisher GUI Node
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
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

    jacobian_velctrl = Node(
        package="ee_velocity_controller",
        executable="jacobian_velctrl_node",
        output="screen",
        parameters=[
            robot_description,
            {
                'base_link': LaunchConfiguration('jacobian_base_link'),
                'ee_link': LaunchConfiguration('jacobian_ee_link'),
                'joint_states_topic': LaunchConfiguration('jacobian_joint_states_topic'),
                'control_rate_hz': LaunchConfiguration('control_rate_hz'),
                'joint_limits_yaml': joint_limits_yaml_path,
                'output_mode': LaunchConfiguration('control_mode'),
                'joint_position_command_topic': '/arm_controller/joint_trajectory',
                'joint_velocity_command_topic': '/arm_velocity_controller/commands',
                'alpha': 0.8, # LPF coefficient for velocity smoothing, between [0, 1). 0 means no smoothing (raw Jacobian output), while closer to 1 means more smoothing
                'use_damped_pseudoinverse': False,
            },
        ],
    )

    return [
        robot_state_publisher_node,
        # joint_state_publisher_node, # don't need this since joint_state_broadcaster publishes to /joint_states
        ros2_control_node,
        *load_controllers,
        static_tf,
        # servo_demo_node,
        velocity_relmove,
        jacobian_velctrl,
        rviz_node,
    ]

def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    launch_args = generate_launch_args()
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld