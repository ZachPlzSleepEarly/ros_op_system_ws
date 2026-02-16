#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

const std::string NODE_NAME = "imu_data_node";
const std::string TOPIC_NAME = "/imu/data";
const int QOS_10 = 10;

std::shared_ptr<rclcpp::Node> node;

void IMUCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    tf2::Quaternion tf2_quaternion;
    tf2_quaternion.setX(msg->orientation.x);
    tf2_quaternion.setY(msg->orientation.y);
    tf2_quaternion.setZ(msg->orientation.z);
    tf2_quaternion.setW(msg->orientation.w);

    tf2::Matrix3x3 matrix(tf2_quaternion);

    double roll;
    double pitch;
    double yaw;
    matrix.getRPY(roll, pitch, yaw);
    roll = roll * 180 / M_PI;
    pitch = pitch * 180 / M_PI;
    yaw = yaw * 180 / M_PI;
    RCLCPP_INFO(node->get_logger(), "roll=%.0f pitch=%0.f yaw=%0.f", roll, pitch, yaw);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>(NODE_NAME);

    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(TOPIC_NAME, QOS_10, IMUCallback);
    rclcpp::spin(node);
    rclcpp::shutdown();
}