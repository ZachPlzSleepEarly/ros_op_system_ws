#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

const std::string NODE_NAME = "lidar_avoidance";
const std::string SCAN_TOPIC_NAME = "/scan";
const std::string VEL_TOPIC_NAME = "/cmd_vel";
const int QOS_10 = 10;

std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
int nCount = 0;

void LidarCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
    int nNum = msg->ranges.size();
    int nMid = nNum / 2;
    float fMidDist = msg->ranges[nMid];
    RCLCPP_INFO(node->get_logger(), "ranges[%d]=%f m", nMid, fMidDist);

    // 正在原地旋转避障中，跳过后面处理
    if (nCount > 0) {
        nCount--;
        return;
    }

    geometry_msgs::msg::Twist vel_msg;
    // 发现障碍物，配置避障操作
    if (fMidDist < 1.5f) {
        vel_msg.angular.z = 3.0;
        nCount = 2;
    } else {
        vel_msg.linear.x = 1.0;
    }

    vel_pub->publish(vel_msg);

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>(NODE_NAME);
    
    auto lidar_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(SCAN_TOPIC_NAME, QOS_10, LidarCallback);
    vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(VEL_TOPIC_NAME, QOS_10);
    rclcpp::spin(node);

    rclcpp::shutdown();
}