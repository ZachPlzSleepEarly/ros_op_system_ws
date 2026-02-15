#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

const std::string NODE_NAME = "velocity_command_node";
const std::string TOPIC_NAME = "/cmd_vel";
const int QOS_10 = 10;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(NODE_NAME);
    auto vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(TOPIC_NAME, QOS_10);
    
    geometry_msgs::msg::Twist vel_msg;
    vel_msg.linear.x = 0.0;  // 右手法则食指
    vel_msg.linear.y = 0.0;  // 右手法则中指
    vel_msg.linear.z = 0.0;  // 右手法则拇指
    vel_msg.angular.x = 0.0;  // 绕右手法则食指轴旋转
    vel_msg.angular.y = 0.0;  // 绕右手法则中指轴旋转
    vel_msg.angular.z = 0.0;  // 绕右手法则拇指轴旋转

    rclcpp::Rate loop_rate(30);
    while (rclcpp::ok()) {
        vel_pub->publish(vel_msg) ;
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}