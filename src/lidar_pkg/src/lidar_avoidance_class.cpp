#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

const std::string NODE_NAME = "lidar_avoidance";
const std::string VEL_TOPIC_NAME = "/cmd_vel";
const std::string LIDAR_TOPIC_NAME = "/scan";
const int QOS_10 = 10;
const int AVOIDANCE_COUNT = 8;

class LidarAvoidance : public rclcpp::Node {
public:
    LidarAvoidance() : rclcpp::Node(NODE_NAME), count_(0)
    {
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(VEL_TOPIC_NAME, QOS_10);
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            LIDAR_TOPIC_NAME, QOS_10, std::bind(&LidarAvoidance::LidarMsgCallback, this, std::placeholders::_1));
    }

private:
    struct MidDistInfo {
        int mid_index;
        float mid_distance;
    };

    void LidarMsgCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
    {

        MidDistInfo mid_info = GetMidDistInfo(msg);
        RCLCPP_INFO(this->get_logger(), "ranges[%d]=%f m", mid_info.mid_index, mid_info.mid_distance);

        if (count_ > 0) {
            count_--;
            return;
        }

        // 1.5米 避障
        geometry_msgs::msg::Twist vel_msg;
        if (mid_info.mid_distance < 1.5f) {
            count_ = AVOIDANCE_COUNT;
            vel_msg.angular.z = 1.5;
        } else {
            vel_msg.linear.x = 0.5;
        }
        vel_pub_->publish(vel_msg);
    }

    MidDistInfo GetMidDistInfo(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
    {
        int nNum = msg->ranges.size();
        int nMid = nNum / 2;
        float midDist = msg->ranges[nMid];
        return {nMid, midDist};
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    int count_ = 0;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = std::make_shared<LidarAvoidance>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}