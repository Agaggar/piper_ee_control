/// \file
/// \brief publish twist commands in end effector frame
///
/// PARAMETERS
/// TODO:
/// PUBLISHERS
/// TODO:
/// SUBSCRIBERS
/// TODO:
/// SERVERS
/// none
/// CLIENTS
/// none

// ros2 topic pub --once /velocity_pub/vel_command ee_velocity_controller/msg/RelativeMove "{dx: 0.1, dy: 0.1, dz: 0.0}"

#include <chrono>
#include <cmath>

/// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
/// message types
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "ee_velocity_controller/msg/relative_move.hpp"

using namespace std::chrono_literals;

class VelPub : public rclcpp::Node
{    
    public:
    VelPub():
        Node("velocity_pub"),
        rate(5.0)
        // more_parameters
        {
            declare_parameter("rate", rclcpp::ParameterValue(rate));
            get_parameter("rate", rate); // get parameter value from "rate", assign to variable rate
            // continue declaring parameters

            velcmd_sub = create_subscription<ee_velocity_controller::msg::RelativeMove>(
                "~/vel_command", 10, std::bind(
                    &VelPub::velcmd_callback, this, std::placeholders::_1
                )
            );
            velcmd_pub = create_publisher<geometry_msgs::msg::TwistStamped>(
                "servo_demo_node/delta_twist_cmds", 100
            );

            timer_ = create_wall_timer(
                std::chrono::milliseconds(int(1.0 / rate * 1000)),
                std::bind(&VelPub::timer_callback, this));
        }
    
    
    private:
        double rate; // other doubles could be rate, param1, param2;
        int numTimesToPublish = 20;
        int currNum = 0;
        rclcpp::Subscription<ee_velocity_controller::msg::RelativeMove>::SharedPtr velcmd_sub;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velcmd_pub;
        // more variables
        geometry_msgs::msg::TwistStamped ee_cmd, zero_cmd;
        rclcpp::TimerBase::SharedPtr timer_;

        //// \brief Timer callback
        void timer_callback() {
            ee_cmd.header.frame_id = "base_link";
            ee_cmd.header.stamp = get_clock()->now();
            zero_cmd.header.frame_id = "base_link";
            zero_cmd.header.stamp = get_clock()->now();
            zero_cmd.twist.linear.x = 0.0;
            zero_cmd.twist.linear.y = 0.0;
            zero_cmd.twist.linear.z = 0.0;
            zero_cmd.twist.angular.x = 0.0;
            zero_cmd.twist.angular.y = 0.0;
            zero_cmd.twist.angular.z = 0.0;
            // if (currNum > 0 && currNum < numTimesToPublish) {
            //     publish_one_sec();
            //     currNum++;
            // }
            // if (currNum == numTimesToPublish) {
            //     currNum = 0;
            // }
            ee_cmd.twist = zero_cmd.twist; // reset ee_cmd to zero
            velcmd_pub->publish(zero_cmd);
        }

        //// \brief subscription callback for custom message
        void velcmd_callback(ee_velocity_controller::msg::RelativeMove::SharedPtr msg) {
            ee_cmd.twist.linear.x = msg->dx;
            ee_cmd.twist.linear.y = msg->dy;
            ee_cmd.twist.linear.z = msg->dz;
            ee_cmd.twist.angular.z = msg->dtheta;
            velcmd_pub->publish(ee_cmd);
            // TODO: map dtheta to angular motion
            // ee_cmd.twist.angular.x = ts->twist.angular.x;
            // ee_cmd.twist.angular.y = ts->twist.angular.y;
            // currNum++;
        }

        //// \brief publish commands for one second
        void publish_one_sec() {
            // no need for initializing, use node value directly
            RCLCPP_INFO(get_logger(), "publishing %d", currNum);
            velcmd_pub->publish(ee_cmd);
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelPub>());
    rclcpp::shutdown();
    return 0;
}