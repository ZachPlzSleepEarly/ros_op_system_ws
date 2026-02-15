#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

const std::string NODE_NAME = "subscriber_class";
const std::string TOPIC_NAME = "/my_topic";
const int QOS_10 = 10;

class SubscriberNode : public rclcpp::Node {
public:
    SubscriberNode() : Node(NODE_NAME)
    {
        subscriber_ = create_subscription<std_msgs::msg::String>(
            TOPIC_NAME, QOS_10, std::bind(&SubscriberNode::Callback, this, std::placeholders::_1));
    }

private:
    void Callback(const std::shared_ptr<const std_msgs::msg::String>& msg)
    {
        RCLCPP_INFO(get_logger(), "Received : %s", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}