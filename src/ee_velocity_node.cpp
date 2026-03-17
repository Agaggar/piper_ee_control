// Following the tutorial here: https://github.com/moveit/moveit2_tutorials/blob/humble/doc/examples/realtime_servo/src/servo_cpp_interface_demo.cpp
// Please see copyright information there

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

    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, "/planning_scene");
    planning_scene_monitor_->providePlanningSceneService();

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

/* Previous chatgpt prompt
I want to write a minimal working C++ node in ROS2 FOXY that uses moveit servo to apply velocity control on the end effector. So far, I have the following pieces, that I need to put together:
servo.yaml:
 # https://github.com/moveit/moveit2/blob/main/moveit_ros/moveit_servo/config/servo_parameters.yaml
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

  ## Publish type
  publish_joint_positions: true
  publish_joint_velocities: true
  publish_joint_accelerations: false

  ## Scaling - max velocity for each
  linear_scale: 0.2
  rotational_scale: 0.5
  joint_scale: 0.5 # only used for unitless

  ## Servo frequency
  publish_period: 0.01

  ## Safety
  incoming_command_timeout: 0.1
  lower_singularity_threshold: 17.0
  hard_stop_singularity_threshold: 30.0

  ## Collision checking
  check_collisions: true
  collision_check_rate: 10.0

robot arm specific configurations, useful for launch:
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
    
    # Controller configuration
    moveit_controllers = load_yaml('piper_moveit_config', 'config/moveit_controllers.yaml')

3. rviz and controllers launch (moveit is unnecessary):
import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml
from launch.actions import TimerAction


def load_yaml(package_name, file_path):
    """Load a YAML file from a ROS package."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    """
    Minimal launch file for Piper robot with MoveIt in simulation.
    
    Equivalent structure to HDT robot's launch file:
    1. Load robot description (XACRO)
    2. Load configurations (controllers, kinematics, etc.)
    3. Start hardware interface (fake)
    4. Start robot state publisher
    5. Start MoveIt move_group
    6. Start RViz
    """
    
    # ==========================================================================
    # Launch Arguments
    # ==========================================================================
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug output'
    )
    
    # ==========================================================================
    # Load Configurations
    # ==========================================================================
    piper_moveit_config_path = get_package_share_directory('piper_moveit_config')

    # ==========================================================================
    # Load Robot Description (from XACRO)
    # ==========================================================================
    piper_description_path = get_package_share_directory('piper_description')
    urdf_file = os.path.join(piper_moveit_config_path, 'config', 'piper.urdf.xacro')
    # urdf_file = os.path.join(piper_description_path, 'urdf', 'piper_description.xacro')
    robot_description_content = subprocess.check_output(['xacro', urdf_file]).decode('utf-8')
    robot_description = {'robot_description': robot_description_content}
    robot_description = {"robot_description": Command(["xacro ", urdf_file])}
    
    
    # Semantic description (SRDF)
    srdf_file = os.path.join(piper_moveit_config_path, 'config', 'piper.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}
    
    # Controller configuration
    moveit_controllers_yaml = load_yaml('piper_moveit_config', 'config/moveit_controllers.yaml') # REAL controllers
    # moveit_controllers_yaml = load_yaml('piper_moveit_config', 'config/fake_moveit_controllers.yaml')
    # controllers_yaml = load_yaml('piper_moveit_config', 'config/ros2_controllers.yaml')
    controllers_yaml_path = os.path.join(piper_moveit_config_path, "config", "ros2_controllers.yaml")
    
    # Kinematics configuration
    kinematics_yaml = load_yaml('piper_moveit_config', 'config/kinematics.yaml')
    
    # Planning configuration
    joint_limits_yaml = load_yaml('piper_moveit_config', 'config/joint_limits.yaml')
    ompl_planning_yaml = load_yaml('piper_moveit_config', 'config/ompl_planning.yaml')
    
    # ==========================================================================
    # Hardware Interface Node (Fake execution for simulation)
    # ==========================================================================
    # fake_hardware_node = Node(
    #     package='piper_moveit_config',
    #     executable='fake_hardware_interface.py',
    #     name='fake_hardware_interface',
    #     output='screen'
    # )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_yaml_path
        ],
        output="screen",
    )
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner.py",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager"
                ],
            )
        ]
    )

    # velocity_controller_spawner = TimerAction(
    #     period=4.0,
    #     actions=[
    #         Node(
    #             package="controller_manager",
    #             executable="spawner.py",
    #             arguments=["joint_group_velocity_controller", "--controller-manager", "/controller_manager"],
    #         )
    #     ]
    # )
    arm_controller_spawner = TimerAction(
        period=5.0,
        actions=[Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"], # for position control
    )])

    arm_controller_velocity_spawner = TimerAction(
        period=5.0,
        actions=[Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["arm_velocity_controller", "--controller-manager", "/controller_manager"], # for velocity control
    )])

    gripper_controller_spawner = TimerAction(
        period=6.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        )]
    )
    
    # ==========================================================================
    # Robot State Publisher (Broadcasts TF from /joint_states)
    # ==========================================================================
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
            {'moveit_simple_controller_manager': moveit_controllers_yaml.get('moveit_simple_controller_manager', {})},
            planning_scene_monitor,
        ]
    )
    
    # ==========================================================================
    # RViz Visualization
    # ==========================================================================
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
    
    # ==========================================================================
    # Launch Description
    # ==========================================================================
    return LaunchDescription([
        debug_arg,
        # fake_hardware_node,
        ros2_control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        arm_controller_velocity_spawner,
        gripper_controller_spawner,
        move_group_node,
        rviz_node,
    ])

Help me write a node in C++ fror ROS2, specifically in FOXY, that uses servo to apply cartesian velocity commands to the end effector
*/