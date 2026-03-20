#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStatePublisher : public rclcpp::Node
{
public:
    JointStatePublisher()
    : Node("minimal_joint_state_pub")
    {
        // Topic you are receiving commands on
        joint_cmd_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/arm_controller/joint_trajectory", 10,
            std::bind(&JointStatePublisher::jointCmdCallback, this, std::placeholders::_1));

        // Joint states publisher (emulates joint_state_broadcaster)
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(5), // 50 ms = 200 Hz
            std::bind(&JointStatePublisher::publishJointState, this));

        // Initialize joint state message
        joint_state_msg_.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}; // Replace with your joints
        joint_state_msg_.position.resize(joint_state_msg_.name.size(), 0.0);
        joint_state_msg_.velocity.resize(joint_state_msg_.name.size(), 0.0);
        joint_state_msg_.effort.resize(joint_state_msg_.name.size(), 0.0);

        RCLCPP_INFO(this->get_logger(), "Minimal JointStatePublisher initialized");
    }

private:
    void jointCmdCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        if (msg->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty trajectory");
            return;
        }

        // For simplicity, take the first point in the trajectory
        const auto& point = msg->points.front();

        // Map positions to joint_state_msg_ based on joint names
        for (size_t i = 0; i < msg->joint_names.size(); ++i)
        {
            auto it = std::find(joint_state_msg_.name.begin(), joint_state_msg_.name.end(), msg->joint_names[i]);
            if (it != joint_state_msg_.name.end())
            {
                size_t idx = std::distance(joint_state_msg_.name.begin(), it);
                joint_state_msg_.position[idx] = point.positions[i];

                // Optionally fill velocities and effort if provided
                if (i < point.velocities.size()) joint_state_msg_.velocity[idx] = point.velocities[i];
                if (i < point.effort.size()) joint_state_msg_.effort[idx] = point.effort[i];
            }
        }

    }
    
    void publishJointState()
    {
        joint_state_msg_.header.stamp = this->get_clock()->now();
        joint_state_pub_->publish(joint_state_msg_);
    }

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    sensor_msgs::msg::JointState joint_state_msg_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStatePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}