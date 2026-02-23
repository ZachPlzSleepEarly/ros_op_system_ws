#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

const std::string NODE_NAME = "point_cloud_node";
const std::string PC_TOPIC_NAME = "/kinect2/sd/points";
const int QOS1 = 1;

class PointCloudNode : public rclcpp::Node {
public:
    PointCloudNode() : rclcpp::Node(NODE_NAME)
    {
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            PC_TOPIC_NAME, QOS1, std::bind(&PointCloudNode::PointCloudCallback, this, std::placeholders::_1));
    }

private:
    void PointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ> pointCloudIn;
        pcl::fromROSMsg(*msg, pointCloudIn);
        int cloudSize = pointCloudIn.points.size();
        for (int i = 0; i < cloudSize; ++i) {
            RCLCPP_INFO(this->get_logger(), "[i=%d] (%.2f, %.2f, %.2f)", 
                i, 
                pointCloudIn.points[i].x,
                pointCloudIn.points[i].y, 
                pointCloudIn.points[i].z);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}