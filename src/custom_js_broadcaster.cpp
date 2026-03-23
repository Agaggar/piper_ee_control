#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStatePublisher : public rclcpp::Node
{
public:
    JointStatePublisher()
    : Node("minimal_joint_state_pub")
    {
        declare_parameter<std::string>("control_mode", "position");
        declare_parameter<bool>("use_gripper", false);

        std::string control_mode;
        bool use_gripper{false};
        get_parameter("control_mode", control_mode);
        get_parameter("use_gripper", use_gripper);

        use_velocity_mode_ = (control_mode == "velocity");

        // Design decision: keep both subscribers alive and gate behavior by control_mode.
        // This allows quick mode switching from launch parameters without recompiling.
        joint_cmd_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "~/arm_position_commands",
            10,
            std::bind(&JointStatePublisher::jointCmdCallback, this, std::placeholders::_1));

        // Receives velocity commands in the same Float64MultiArray format published by
        // jacobian_velctrl::publish_joint_velocity_command().
        arm_velocity_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "~/arm_velocity_commands",
            10,
            std::bind(&JointStatePublisher::armVelocityCmdCallback, this, std::placeholders::_1));

        // Joint states publisher (emulates joint_state_broadcaster)
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(5), // 50 ms = 200 Hz
            std::bind(&JointStatePublisher::publishJointState, this));

        // Initialize joint state message
        if (use_gripper) {
            joint_state_msg_.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"};
        } else {
            joint_state_msg_.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        }
        joint_state_msg_.position.resize(joint_state_msg_.name.size(), 0.0);
        joint_state_msg_.velocity.resize(joint_state_msg_.name.size(), 0.0);
        joint_state_msg_.effort.resize(joint_state_msg_.name.size(), 0.0);

        RCLCPP_INFO(
            this->get_logger(),
            "Minimal JointStatePublisher initialized (control_mode='%s', joints=%zu)",
            use_velocity_mode_ ? "velocity" : "position",
            joint_state_msg_.name.size());
    }

private:
    void jointCmdCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        if (use_velocity_mode_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Ignoring JointTrajectory command while in velocity mode.");
            return;
        }

        if (msg->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty trajectory");
            return;
        }

        std::lock_guard<std::mutex> lock(state_mutex_);

        // For simplicity, take the first point in the trajectory
        const auto& point = msg->points.front();

        // Map positions to joint_state_msg_ based on joint names
        for (size_t i = 0; i < msg->joint_names.size(); ++i)
        {
            auto it = std::find(joint_state_msg_.name.begin(), joint_state_msg_.name.end(), msg->joint_names[i]);
            if (it != joint_state_msg_.name.end())
            {
                size_t idx = std::distance(joint_state_msg_.name.begin(), it);
                if (i < point.positions.size()) {
                    joint_state_msg_.position[idx] = point.positions[i];
                }

                // Optionally fill velocities and effort if provided
                if (i < point.velocities.size()) joint_state_msg_.velocity[idx] = point.velocities[i];
                if (i < point.effort.size()) joint_state_msg_.effort[idx] = point.effort[i];
            }
        }

    }

    void armVelocityCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (!use_velocity_mode_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Ignoring Float64MultiArray velocity command while in position mode.");
            return;
        }

        if (msg->data.empty()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Received empty arm velocity command.");
            return;
        }

        std::lock_guard<std::mutex> lock(state_mutex_);

        const size_t joint_count = joint_state_msg_.name.size();
        const size_t copy_count = std::min(joint_count, msg->data.size());

        // Design decision: map incoming array values by index to joint_state_msg_.name order.
        // For current jacobian_velctrl output this means the first 6 values feed joint1..joint6.
        for (size_t index = 0; index < copy_count; ++index) {
            joint_state_msg_.velocity[index] = msg->data[index];
        }

        // Any joints not included in the command (e.g., gripper joints 7/8) are held at zero velocity.
        for (size_t index = copy_count; index < joint_count; ++index) {
            joint_state_msg_.velocity[index] = 0.0;
        }

        has_velocity_command_ = true;
    }
    
    void publishJointState()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        const rclcpp::Time now = this->get_clock()->now();
        if (use_velocity_mode_ && has_velocity_command_ && has_last_publish_time_) {
            const double dt = (now - last_publish_time_).seconds();
            if (dt > 0.0) {
                for (size_t index = 0; index < joint_state_msg_.position.size(); ++index) {
                    joint_state_msg_.position[index] += joint_state_msg_.velocity[index] * dt;
                }
            }
        }

        last_publish_time_ = now;
        has_last_publish_time_ = true;
        joint_state_msg_.header.stamp = this->get_clock()->now();
        joint_state_pub_->publish(joint_state_msg_);
    }

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr arm_velocity_cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    sensor_msgs::msg::JointState joint_state_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool use_velocity_mode_{false};
    bool has_velocity_command_{false};
    bool has_last_publish_time_{false};
    rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
    std::mutex state_mutex_;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStatePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}