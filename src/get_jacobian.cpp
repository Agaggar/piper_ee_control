#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class JacobianNode : public rclcpp::Node
{
public:
    JacobianNode()
    : Node("get_jacobian_node")
    {
        declare_parameter<std::string>("robot_description", "");
        declare_parameter<std::string>("urdf_path", "");
        declare_parameter<std::string>("base_link", "base_link");
        declare_parameter<std::string>("ee_link", "link6");
        declare_parameter<std::string>("joint_states_topic", "/joint_states");
        declare_parameter<double>("compute_rate_hz", 20.0);

        std::string robot_description;
        std::string urdf_path;
        std::string base_link;
        std::string ee_link;
        std::string joint_states_topic;
        double compute_rate_hz;

        get_parameter("robot_description", robot_description);
        get_parameter("urdf_path", urdf_path);
        get_parameter("base_link", base_link);
        get_parameter("ee_link", ee_link);
        get_parameter("joint_states_topic", joint_states_topic);
        get_parameter("compute_rate_hz", compute_rate_hz);

        if (!initialize_kdl(robot_description, urdf_path, base_link, ee_link)) {
            throw std::runtime_error("Failed to initialize KDL chain/solver.");
        }

        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic,
            20,
            std::bind(&JacobianNode::joint_state_callback, this, std::placeholders::_1));

        // Design decision: publish Jacobian matrices as Float64MultiArray (row-major)
        // to avoid introducing a new custom message type for this utility node.
        jacobian_pub_ =
            create_publisher<std_msgs::msg::Float64MultiArray>("~/jacobian", 10);
        jacobian_pinv_pub_ =
            create_publisher<std_msgs::msg::Float64MultiArray>("~/jacobian_pinv", 10);

        const auto timer_period = std::chrono::duration<double>(1.0 / std::max(compute_rate_hz, 1.0));
        compute_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
            std::bind(&JacobianNode::compute_and_publish, this));

        RCLCPP_INFO(
            get_logger(),
            "Initialized Jacobian node with base_link='%s', ee_link='%s', joints=%zu",
            base_link.c_str(), ee_link.c_str(), chain_joint_names_.size());
    }

private:
    bool initialize_kdl(
        const std::string & robot_description,
        const std::string & urdf_path,
        const std::string & base_link,
        const std::string & ee_link)
    {
        // Design decision: prefer `robot_description` first because the bringup launch
        // already provides xacro-expanded URDF, and this avoids hard-coding package paths.
        const bool parsed_from_description =
            !robot_description.empty() && kdl_parser::treeFromString(robot_description, kdl_tree_);
        const bool parsed_from_file =
            parsed_from_description ? true : (!urdf_path.empty() && kdl_parser::treeFromFile(urdf_path, kdl_tree_));

        if (!parsed_from_description && !parsed_from_file) {
            RCLCPP_ERROR(
                get_logger(),
                "Could not parse URDF. Set 'robot_description' or valid 'urdf_path'.");
            return false;
        }

        if (!kdl_tree_.getChain(base_link, ee_link, kdl_chain_)) {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to create KDL chain from '%s' to '%s'.",
                base_link.c_str(), ee_link.c_str());
            return false;
        }

        jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_chain_);
        chain_joint_names_ = extract_chain_joint_names(kdl_chain_);

        if (chain_joint_names_.empty()) {
            RCLCPP_ERROR(get_logger(), "No non-fixed joints found in the selected KDL chain.");
            return false;
        }

        return true;
    }

    static std::vector<std::string> extract_chain_joint_names(const KDL::Chain & chain)
    {
        std::vector<std::string> names;
        names.reserve(chain.getNrOfJoints());
        for (const auto & segment : chain.segments) {
            const auto & joint = segment.getJoint();
            if (joint.getType() != KDL::Joint::None) {
                names.push_back(joint.getName());
            }
        }
        return names;
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        latest_joint_state_ = msg;
    }

    void compute_and_publish()
    {
        sensor_msgs::msg::JointState::SharedPtr joint_state_snapshot;
        {
            std::lock_guard<std::mutex> lock(joint_state_mutex_);
            joint_state_snapshot = latest_joint_state_;
        }

        if (!joint_state_snapshot || joint_state_snapshot->position.empty()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Waiting for valid /joint_states data...");
            return;
        }

        std::unordered_map<std::string, double> joint_positions;
        const size_t count = std::min(joint_state_snapshot->name.size(), joint_state_snapshot->position.size());
        joint_positions.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            joint_positions[joint_state_snapshot->name[i]] = joint_state_snapshot->position[i];
        }

        KDL::JntArray kdl_joint_positions(kdl_chain_.getNrOfJoints());
        for (size_t index = 0; index < chain_joint_names_.size(); ++index) {
            const auto & joint_name = chain_joint_names_[index];
            const auto found = joint_positions.find(joint_name);
            if (found == joint_positions.end()) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Joint '%s' not found in /joint_states.",
                    joint_name.c_str());
                return;
            }
            kdl_joint_positions(index) = found->second;
        }

        KDL::Jacobian kdl_jacobian(kdl_chain_.getNrOfJoints());
        const int status = jacobian_solver_->JntToJac(kdl_joint_positions, kdl_jacobian);
        if (status < 0) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "KDL Jacobian solver failed with status=%d",
                status);
            return;
        }

        Eigen::MatrixXd jacobian_matrix = kdl_jacobian.data;
        Eigen::MatrixXd jacobian_pseudoinverse =
            jacobian_matrix.completeOrthogonalDecomposition().pseudoInverse();

        jacobian_pub_->publish(to_multi_array(jacobian_matrix));
        jacobian_pinv_pub_->publish(to_multi_array(jacobian_pseudoinverse));
    }

    static std_msgs::msg::Float64MultiArray to_multi_array(const Eigen::MatrixXd & matrix)
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.layout.dim.resize(2);
        msg.layout.dim[0].label = "rows";
        msg.layout.dim[0].size = matrix.rows();
        msg.layout.dim[0].stride = matrix.rows() * matrix.cols();
        msg.layout.dim[1].label = "cols";
        msg.layout.dim[1].size = matrix.cols();
        msg.layout.dim[1].stride = matrix.cols();
        msg.data.reserve(static_cast<size_t>(matrix.rows() * matrix.cols()));

        for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
            for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
                msg.data.push_back(matrix(row, col));
            }
        }
        return msg;
    }

    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;
    std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
    std::vector<std::string> chain_joint_names_;

    sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
    std::mutex joint_state_mutex_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr jacobian_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr jacobian_pinv_pub_;
    rclcpp::TimerBase::SharedPtr compute_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JacobianNode>());
    rclcpp::shutdown();
    return 0;
}

// double lambda = 0.01;

// Eigen::MatrixXd J_pinv =
//     J.transpose() *
//     (J * J.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(6,6)).inverse();