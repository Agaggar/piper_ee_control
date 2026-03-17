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
    # ==========================================================================
    # Launch Arguments
    # ==========================================================================
    moveit_arg = DeclareLaunchArgument(
        name='use_moveit',
        default_value='true',
        description='Enable MoveIt'
    )
    # ==========================================================================
    # Load Configurations
    # ==========================================================================
    piper_moveit_config_path = get_package_share_directory('piper_moveit_config')
    
    moveit_cpp_yaml_file_name = (get_package_share_directory("ee_velocity_controller") + "/config/moveit_cpp.yaml")
    # xacro
    # Component yaml files are grouped in separate namespaces
    robot_description_config = xacro.process_file(
        os.path.join(piper_moveit_config_path, 'config', 'piper.urdf.xacro'),
        mappings={"initial_positions_file": os.path.join(piper_moveit_config_path, "config", "initial_positions.yaml")})
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

    # Controller configuration
    moveit_controllers = load_yaml('piper_moveit_config', 'config/moveit_controllers.yaml')
    controllers_yaml_path = os.path.join(piper_moveit_config_path, "config", "ros2_controllers.yaml")

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
    servo_params = (
        ParameterBuilder("ee_velocity_controller")
        .yaml(
            parameter_namespace="moveit_servo",
            file_path="config/piper_simulated_servo_params.yaml",
        )
        .to_dict()
    )
    # A node to publish world -> base_link transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )
    servo_demo_node = Node(
        package="ee_velocity_controller",
        executable="servo_demo_node",
        output="screen",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
        ],
    )

    # control nodes
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
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
        ]
    )
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"publish_rate": 200}]
    )
    
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
        # moveit_arg,
        # robot_state_publisher_node,
        # ros2_control_node,
        # *load_controllers,
        # move_group_node,
        # joint_state_publisher,
        # rviz_node,
        # servo_node
        servo_demo_node,
        # static_tf
    ])

"""
previous chatgpt prompt:
I'm trying to use moveit servo ros2 FOXY to move the end effector of a robot. i can paste in necessary yaml or config files if you need, but here is a launch file that launches the robot, along with the C++ that spins up servo, the topic i am publishing to, and the servo.yaml.

piper_servo.yaml:
 # https://github.com/moveit/moveit2/blob/main/moveit_ros/moveit_servo/config/servo_parameters.yaml
minimal_servo_node:
  ros__parameters:
    moveit_servo:
      ## MoveIt group
      move_group_name: arm

      ## The link that will be controlled
      ee_frame_name: gripper_base

      ## Robot base frame
      planning_frame: base_link

      ## Command input type
      command_in_type: speed_units

      ## Topics for incoming commands
      cartesian_command_in_topic: ~/delta_twist_cmds
      joint_command_in_topic: ~/delta_joint_cmds

      ## Output
      command_out_topic: /arm_controller/joint_trajectory
      command_out_type: trajectory_msgs/JointTrajectory # don't change this
      status_topic: ~/status

      # Planning
      joint_topic: /joint_states
      monitored_planning_scene_topic: /planning_scene
      check_octomap_collisions: false

      robot_link_command_frame: base_link

      scale:
        linear: 0.2
        rotational: 0.5
        joint: 0.5

      publish_period: 0.01

      low_pass_filter_coeff: 2.0

      incoming_command_timeout: 0.1

      publish_joint_positions: true
      publish_joint_velocities: true
      publish_joint_accelerations: false

      lower_singularity_threshold: 17.0
      hard_stop_singularity_threshold: 30.0

      check_collisions: true
      collision_check_rate: 10.0
      collision_check_type: 'threshold_distance'
      self_collision_proximity_threshold: 0.001
      scene_collision_proximity_threshold: 0.001
      collision_distance_safety_factor: 1.0
      min_allowable_collision_distance: 0.001

launch file:
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command, LaunchConfiguration
# from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os, subprocess
import yaml

import xacro


def load_yaml(package_name, file_path):
    '''Load a YAML file from a ROS package.'''
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
    # Launch Arguments
    # ==========================================================================
    moveit_arg = DeclareLaunchArgument(
        name='use_moveit',
        default_value='true',
        description='Enable MoveIt'
    )
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
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        },
    }
    ompl_planning_yaml = load_yaml('piper_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)
    joint_limits_yaml = load_yaml('piper_moveit_config', 'config/joint_limits.yaml')

    # Controller configuration
    moveit_controllers = load_yaml('piper_moveit_config', 'config/moveit_controllers.yaml')
    controllers_yaml_path = os.path.join(piper_moveit_config_path, "config", "ros2_controllers.yaml")

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

    # control nodes
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
    controllers = ["joint_state_broadcaster", "arm_controller", "arm_velocity_controller", "gripper_controller"]
    for controller in controllers:
        load_controllers += [
            TimerAction(
            period=5.0,
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
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
        ]
    )
    
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
        moveit_arg,
        robot_state_publisher_node,
        ros2_control_node,
        *load_controllers,
        move_group_node,
        rviz_node,
        servo_node
    ])

C++ servo node:
#include <rclcpp/rclcpp.hpp>

#include <moveit_servo/servo.h>
#include <moveit_servo/servo_parameters.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

class MinimalServoNode : public rclcpp::Node
{
public:
  MinimalServoNode()
  : Node("minimal_servo_node")
  {
    RCLCPP_INFO(get_logger(), "Node created");
  }

  void init()
  {
    RCLCPP_INFO(get_logger(), "Initializing MoveIt Servo");

    //---------------------------------------------
    // Load robot model
    //---------------------------------------------
    robot_model_loader_ =
        std::make_shared<robot_model_loader::RobotModelLoader>(
            shared_from_this(),
            "robot_description");

    robot_model_ = robot_model_loader_->getModel();

    //---------------------------------------------
    // Planning scene monitor
    //---------------------------------------------
    planning_scene_monitor_ =
        std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            shared_from_this(),
            robot_model_loader_);

    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();

    //---------------------------------------------
    // Load servo parameters
    //---------------------------------------------
    servo_params_ =
        moveit_servo::ServoParameters::makeServoParameters(
            shared_from_this(),
            get_logger(),
            "moveit_servo",
            false);

    if (!servo_params_)
    {
      RCLCPP_ERROR(get_logger(), "Failed to load servo parameters");
      return;
    }
    else {
      RCLCPP_INFO(get_logger(), "Servo parameters loaded");
    }

    //---------------------------------------------
    // Create Servo object
    //---------------------------------------------
    servo_ = std::make_unique<moveit_servo::Servo>(
        shared_from_this(),
        servo_params_,
        planning_scene_monitor_);

    servo_->start();

    RCLCPP_INFO(get_logger(), "Servo started and listening for commands");
  }

private:

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  moveit_servo::ServoParameters::SharedConstPtr servo_params_;

  std::unique_ptr<moveit_servo::Servo> servo_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MinimalServoNode>();

  node->init();   // <-- important

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

command line topic pub:
$ ros2 topic pub -r 100 /minimal_servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "
header:
  frame_id: base_link
twist:
  linear:
    x: 0.05
    y: 0
    z: 0
  angular:
    x: 0
    y: 0
    z: 0"

everything runs without an error, but the robot doesn't move in rviz. what are the steps to debug here?
"""