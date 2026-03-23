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
