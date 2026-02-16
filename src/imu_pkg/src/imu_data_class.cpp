#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

const std::string NODE_NAME = "imu_data_node";
const std::string IMU_TOPIC_NAME = "/imu/data";
const int QOS_10 = 10;

class ImuDataSub : public rclcpp::Node {
public:
    ImuDataSub() : rclcpp::Node(NODE_NAME)
    {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            IMU_TOPIC_NAME, QOS_10, std::bind(&ImuDataSub::ImuCallback, this, std::placeholders::_1));
    }

private:
    void ImuCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg) 
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

        RCLCPP_INFO(this->get_logger(), "roll=%.0f pitch=%.0f yaw=%.0f", roll, pitch, yaw);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr node = std::make_shared<ImuDataSub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}