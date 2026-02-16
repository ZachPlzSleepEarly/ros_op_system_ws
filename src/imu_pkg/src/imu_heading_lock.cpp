#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>

const std::string NODE_NAME = "imu_heading_lock";
const std::string IMU_TOPIC_NAME = "/imu/data";
const std::string VEL_TOPIC_NAME = "/cmd_vel";
const int QOS_10 = 10;
const double TARGET_YAW = 90;
const double RAW_CONTROL_GAIN = 0.01;
const double SPEED_X = 0.5;

std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

void ImuCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    tf2::Quaternion tf2_quaternion;
    tf2_quaternion.setX(msg->orientation.x);
    tf2_quaternion.setY(msg->orientation.y);
    tf2_quaternion.setZ(msg->orientation.z);
    tf2_quaternion.setW(msg->orientation.w);

    tf2::Matrix3x3 rotation_matrix(tf2_quaternion);
    double roll;
    double pitch;
    double yaw;
    rotation_matrix.getRPY(roll, pitch, yaw);
    roll = roll * 180 / M_PI;
    pitch = pitch * 180 / M_PI;
    yaw = yaw * 180 / M_PI;
    RCLCPP_INFO(node->get_logger(), "roll=%.0f pitch=%.0f yaw=%.0f", roll, pitch, yaw);

    double diffAngle = TARGET_YAW - yaw;
    geometry_msgs::msg::Twist vel_msg;
    vel_msg.angular.z = diffAngle * RAW_CONTROL_GAIN;
    vel_msg.linear.x = SPEED_X;
    vel_pub->publish(vel_msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>(NODE_NAME);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC_NAME, QOS_10, ImuCallback);
    vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(VEL_TOPIC_NAME, QOS_10);
    rclcpp::spin(node);
    rclcpp::shutdown();
}