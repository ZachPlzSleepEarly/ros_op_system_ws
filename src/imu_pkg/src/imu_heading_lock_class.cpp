#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>

const std::string NODE_NAME = "imu_heading_lock_node";
const std::string VEL_TOPIC_NAME = "/cmd_vel";
const std::string IMU_TOPIC_NAME = "/imu/data";
const int QOS_10 = 10;

class ImuHeadingLock : public rclcpp::Node {
public:
    ImuHeadingLock() : rclcpp::Node(NODE_NAME)
    {
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(VEL_TOPIC_NAME, QOS_10);
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            IMU_TOPIC_NAME, QOS_10, std::bind(&ImuHeadingLock::ImuCallback, this, std::placeholders::_1));
    }

private:
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
        RCLCPP_INFO(this->get_logger(), "roll=%.0f pitch=%.0f yaw=%.0f", roll, pitch, yaw);

        double diff_yaw = TARGET_YAW_ANGLE_ - yaw;
        geometry_msgs::msg::Twist vel_msg;
        vel_msg.angular.z = diff_yaw * YAW_CONTROL_RATE_;
        vel_msg.linear.x = SPEED_X_;
        vel_pub_->publish(vel_msg);
    }

    const double TARGET_YAW_ANGLE_ = 90;
    const double YAW_CONTROL_RATE_ = 0.006;
    const double SPEED_X_ = 1.5;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuHeadingLock>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}