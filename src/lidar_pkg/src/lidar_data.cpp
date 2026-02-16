#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

const std::string NODE_NAME = "lidar_data_node";
const std::string TOPIC_NAME = "/scan";
const int QOS_10 = 10;

std::shared_ptr<rclcpp::Node> node;  // 为了main和回调都能访问，因此声明全局变量

void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    int nNum = msg->ranges.size();

    int nMid = nNum / 2;
    float fMidDist = msg->ranges[nMid];
    RCLCPP_INFO(node->get_logger(), "ranges[%d]=%f m", nMid, fMidDist);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>(NODE_NAME);
    
    auto lidar_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(TOPIC_NAME, QOS_10, LidarCallback);
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}