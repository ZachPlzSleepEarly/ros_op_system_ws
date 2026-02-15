#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

const std::string NODE_NAME = "publisher_class";
const std::string TOPIC_NAME = "/my_topic";
const int QOS_10 = 10;

class PublisherNode : public rclcpp::Node {
public:
    PublisherNode() : Node(NODE_NAME)
    {
        publisher_ = create_publisher<std_msgs::msg::String>(TOPIC_NAME, QOS_10);
        timer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&PublisherNode::PublishMessage, this));
    }

private:
    void PublishMessage()
    {
        message_.data = "Hello World";
        publisher_->publish(message_);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std_msgs::msg::String message_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PublisherNode>();
    RCLCPP_INFO(node->get_logger(), "Start publishing ...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}