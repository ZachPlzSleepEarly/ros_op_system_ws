#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

const std::string NODE_NAME = "waypoint_navigation_node";
const std::string PUB_TOPIC_NAME = "/waterplus/navi_waypoint";
const std::string RESULT_TOPIC_NAME = "/waterplus/navi_result";
const std::string NAVI_FINISH_STATE = "navi done";
const int QOS10 = 10;

std::shared_ptr<rclcpp::Node> node;

void ResultCallback(std_msgs::msg::String::ConstSharedPtr msg)
{
    if (msg->data == NAVI_FINISH_STATE) {
        RCLCPP_INFO(node->get_logger(), "Arrived!");
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>(NODE_NAME);
    auto navigation_pub = node->create_publisher<std_msgs::msg::String>(PUB_TOPIC_NAME, QOS10);
    auto result_sub = node->create_subscription<std_msgs::msg::String>(RESULT_TOPIC_NAME, QOS10, ResultCallback);

    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    std_msgs::msg::String waypoint_msg;
    waypoint_msg.data = "1";
    navigation_pub->publish(waypoint_msg);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}