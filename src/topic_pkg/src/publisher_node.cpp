#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

const std::string NODE_NAME = "publisher_node";
const std::string TOPIC_NAME = "/my_topic";
const int QOS = 10;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(NODE_NAME);
    auto publisher = node->create_publisher<std_msgs::msg::String>(NODE_NAME, QOS);

    std_msgs::msg::String message;
    message.data = "Hello World";

    rclcpp::Rate loop_rate(1);  // 在 while 被调用，控制 while 每 1s 执行一次

    while (rclcpp::ok()) {
        publisher->publish(message);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}