#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

const std::string NODE_NAME = "lidar_data_node";
const std::string TOPIC_NAME = "/scan";
const int QOS_10 = 10;

class LidarDataSubscriber : public rclcpp::Node {
public:
    LidarDataSubscriber() : rclcpp::Node(NODE_NAME)
    {
        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            TOPIC_NAME, QOS_10, std::bind(&LidarDataSubscriber::LidarCallback, this, std::placeholders::_1));
    }

private:
    void LidarCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
    {
        int mid = msg->ranges.size() / 2;
        RCLCPP_INFO(this->get_logger(), "ranges[%d]=%f m", mid, msg->ranges[mid]);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<LidarDataSubscriber> node = std::make_shared<LidarDataSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}