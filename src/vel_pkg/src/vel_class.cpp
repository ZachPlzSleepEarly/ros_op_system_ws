#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

const std::string NODE_NAME = "velocity_command_node";
const std::string TOPIC_NAME = "/cmd_vel";
const int CONTROL_LOOP_HZ = 30;
const int QOS_10 = 10;

class VelCommandNode : public rclcpp::Node {
public:
    VelCommandNode() : rclcpp::Node(NODE_NAME)
    {
        publisher_ = create_publisher<geometry_msgs::msg::Twist>(TOPIC_NAME, QOS_10);
        timer_ = create_wall_timer(std::chrono::milliseconds(CONTROL_LOOP_HZ), std::bind(&VelCommandNode::PublishVelMsg, this));
    }

private:
    void PublishVelMsg()
    {
        vel_msg_.linear.x = 1.0;
        vel_msg_.linear.y = 0.0;
        vel_msg_.linear.z = 0.0;
        vel_msg_.angular.x = 0.0;
        vel_msg_.angular.y = 0.0;
        vel_msg_.angular.z = 0.0;

        publisher_->publish(vel_msg_);
    }


    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist vel_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<VelCommandNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}