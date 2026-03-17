// modifying servo_cpp_interface_demo.cpp from realtime_servo https://github.com/moveit/moveit2_tutorials/blob/humble/doc/examples/realtime_servo/src/servo_cpp_interface_demo.cpp
// please see copyright information at the github website


// ROS
#include <rclcpp/rclcpp.hpp>

// Servo
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("servo demo");


// BEGIN_TUTORIAL

// Setup
// ^^^^^
// First we declare pointers to the node and publisher that will publish commands to Servo
rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmd_pub_;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;
size_t count_ = 0;

// BEGIN_SUB_TUTORIAL publishCommands
// Here is the timer callback for publishing commands. The C++ interface sends commands through internal ROS topics,
// just like if Servo was launched using ServoNode.
void publishCommands()
{
  // First we will publish 100 joint jogging commands. The :code:`joint_names` field allows you to specify individual
  // joints to move, at the velocity in the corresponding :code:`velocities` field. It is important that the message
  // contains a recent timestamp, or Servo will think the command is stale and will not move the robot.
  RCLCPP_INFO(LOGGER, "At count %zu, publishing commands", count_);
  if (count_ < 20)
  {
     auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = node_->now();
    msg->header.frame_id = "base_link";
    msg->twist.linear.z = 0.1;
    // msg->twist.angular.z = 0.5;
    twist_cmd_pub_->publish(std::move(msg));
    ++count_;
  }
  else if (20 <= count_ && count_ < 40)
  {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = node_->now();
    msg->header.frame_id = "base_link";
    msg->twist.linear.x = 0.2;
    // msg->twist.angular.z = 0.5;
    twist_cmd_pub_->publish(std::move(msg));
    ++count_;
  }
  else if (40 <= count_ && count_ < 60)
  {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = node_->now();
    msg->header.frame_id = "base_link";
    msg->twist.linear.y = -0.2;
    // msg->twist.angular.z = 0.5;
    twist_cmd_pub_->publish(std::move(msg));
    ++count_;
  }
  else if (60 <= count_ && count_ < 80)
  {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = node_->now();
    msg->header.frame_id = "base_link";
    msg->twist.linear.x = -0.1;
    msg->twist.linear.y = 0.;
    // msg->twist.angular.z = 0.5;
    twist_cmd_pub_->publish(std::move(msg));
    ++count_;
  }
  else if (80 <= count_ && count_ < 100)
  {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = node_->now();
    msg->header.frame_id = "base_link";
    msg->twist.linear.y = 0.2;
    // msg->twist.angular.z = 0.5;
    twist_cmd_pub_->publish(std::move(msg));
    ++count_;
  }
  // After a while, we switch to publishing twist commands. The provided frame is the frame in which the twist is
  // defined, not the robot frame that will follow the command. Again, we need a recent timestamp in the message
  else if (count_ == 100)
  {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = node_->now();
    msg->header.frame_id = "base_link";
    msg->twist.linear.x = 0.;
    // msg->twist.angular.z = 0.5;
    twist_cmd_pub_->publish(std::move(msg));
  }
}
// END_SUB_TUTORIAL


// Next we will set up the node, planning_scene_monitor, and collision object
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  // This is false for now until we fix the QoS settings in moveit to enable intra process comms
  node_options.use_intra_process_comms(false);
  node_ = std::make_shared<rclcpp::Node>("servo_demo_node", node_options);

  // Pause for RViz to come up. This is necessary in an integrated demo with a single launch file
  rclcpp::sleep_for(std::chrono::seconds(6));

  // Create the planning_scene_monitor. We need to pass this to Servo's constructor, and we should set it up first
  // before initializing any collision objects
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_, "robot_description", tf_buffer, "planning_scene_monitor");

  // Here we make sure the planning_scene_monitor is updating in real time from the joint states topic
  if (planning_scene_monitor->getPlanningScene())
  {
    planning_scene_monitor->startStateMonitor("/joint_states");
    planning_scene_monitor->setPlanningScenePublishingFrequency(25);
    planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                         "/moveit_servo/publish_planning_scene");
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->providePlanningSceneService();
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning scene not configured");
    return EXIT_FAILURE;
  }

  // These are the publishers that will send commands to MoveIt Servo. Two command types are supported: JointJog
  // messages which will directly jog the robot in the joint space, and TwistStamped messages which will move the
  // specified link with the commanded Cartesian velocity. In this demo, we jog the end effector link.
  joint_cmd_pub_ = node_->create_publisher<control_msgs::msg::JointJog>("servo_demo_node/delta_joint_cmds", 10);
  twist_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("servo_demo_node/delta_twist_cmds", 10);

  /*
  // Next we will create a collision object in the way of the arm. As the arm is servoed towards it, it will slow down
  // and stop before colliding
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = "box";

  // Make a box and put it in the way
  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = { 0.1, 0.4, 0.3 };
  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 0.4;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.3;

  // Add the box as a collision object
  collision_object.primitives.push_back(box);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // Create the message to publish the collision object
  moveit_msgs::msg::PlanningSceneWorld psw;
  psw.collision_objects.push_back(collision_object);
  moveit_msgs::msg::PlanningScene ps;
  ps.is_diff = true;
  ps.world = psw;

  // Publish the collision object to the planning scene
  auto scene_pub = node_->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
  scene_pub->publish(ps);
  */

  // Initializing Servo
  // ^^^^^^^^^^^^^^^^^^
  // Servo requires a number of parameters to dictate its behavior. These can be read automatically by using the
  // :code:`makeServoParameters` helper function
  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node_,
            LOGGER,
            "moveit_servo",
            false);
  if (!servo_parameters)
  {
    RCLCPP_FATAL(LOGGER, "Failed to load the servo parameters");
    return EXIT_FAILURE;
  }

  // Initialize the Servo C++ interface by passing a pointer to the node, the parameters, and the PSM
  auto servo = std::make_unique<moveit_servo::Servo>(node_, servo_parameters, planning_scene_monitor);

  // You can start Servo directly using the C++ interface. If launched using the alternative ServoNode, a ROS
  // service is used to start Servo. Before it is started, MoveIt Servo will not accept any commands or move the robot
  servo->start();

  // Sending Commands
  // ^^^^^^^^^^^^^^^^
  // For this demo, we will use a simple ROS timer to send joint and twist commands to the robot
  // rclcpp::TimerBase::SharedPtr timer = node_->create_wall_timer(100ms, publishCommands);

  // CALL_SUB_TUTORIAL publishCommands

  // We use a multithreaded executor here because Servo has concurrent processes for moving the robot and avoiding collisions
  auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node_);
  executor->spin();

  // END_TUTORIAL

  rclcpp::shutdown();
  return 0;
}

/* 
class MoveItCppDemo
{
public:
  MoveItCppDemo(const rclcpp::Node::SharedPtr& node)
    : node_(node)
    , robot_state_publisher_(node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1))
  {
  }

  void run()
  {
    RCLCPP_INFO(LOGGER, "Initialize MoveItCpp");
    moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(node_);
    moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();  // let RViz display query PlanningScene
    moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);

    RCLCPP_INFO(LOGGER, "Initialize PlanningComponent");
    moveit_cpp::PlanningComponent arm("arm", moveit_cpp_);

    // A little delay before running the plan
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Create collision object, planning shouldn't be too easy
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.id = "box";

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = { 0.1, 0.4, 0.1 };

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.4;
    box_pose.position.y = 0.0;
    box_pose.position.z = 1.0;

    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Add object to planning scene
    {  // Lock PlanningScene
      planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
      scene->processCollisionObjectMsg(collision_object);
    }  // Unlock PlanningScene

    // Set joint state goal
    RCLCPP_INFO(LOGGER, "Set goal");
//     std::map<std::string, double> target{
//   {"joint1", 0.0},
//   {"joint2", 0.5},
//   {"joint3", 1.0},
//   {"joint4", 0.0},
//   {"joint5", 0.5},
//   {"joint6", 0.0}
// };
    arm.setGoal("zero");
    // auto current_state = moveit_cpp_->getCurrentState();
    // arm.setStartState(*current_state);
    // arm.setGoal(*current_state);

    // Run actual plan
    RCLCPP_INFO(LOGGER, "Plan to goal");
    const auto plan_solution = arm.plan();
    if (plan_solution)
    {
      RCLCPP_INFO(LOGGER, "arm.execute()");
      arm.execute();
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  moveit_cpp::MoveItCppPtr moveit_cpp_;
};

int main(int argc, char** argv)
{
  RCLCPP_INFO(LOGGER, "Initialize node");
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);

  MoveItCppDemo demo(node);
  std::thread run_demo([&demo]() {
    // Let RViz initialize before running demo
    // TODO(henningkayser): use lifecycle events to launch node
    rclcpp::sleep_for(std::chrono::seconds(5));
    demo.run();
  });

  rclcpp::spin(node);
  run_demo.join();

  return 0;
}

*/