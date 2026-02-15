#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

const std::string NODE_NAME = "subscriber_node";
const std::string TOPIC_NAME = "/my_topic";
const int QOS = 10;
std::shared_ptr<rclcpp::Node> node;

void Callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), "Receive : %s", msg->data.c_str());
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>(NODE_NAME);
    auto subscriber = node->create_subscription<std_msgs::msg::String>(TOPIC_NAME, QOS, &Callback);
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}