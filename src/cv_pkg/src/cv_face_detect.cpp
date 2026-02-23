#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <geometry_msgs/msg/twist.hpp>

const std::string NODE_NAME = "cv_face_detect_node";
const std::string CAM_TOPIC_NAME = "/kinect2/qhd/image_raw";
const std::string FRAME_TOPIC_NAME = "/face_detector_input";
const std::string FACE_TOPIC_NAME = "/face_position";
const std::string VEL_TOPIC_NAME = "/cmd_vel";
const std::string FACE_WIN_NAME = "Face";
const int QOS1 = 1;
const int QOS10 = 10;

static std::shared_ptr<rclcpp::Node> node;
static rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr frame_pub;
static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
static cv::Mat imgFace;
static geometry_msgs::msg::Twist vel_cmd;
static double Kp = 0.002;

void DrawRectangle(sensor_msgs::msg::RegionOfInterest::ConstSharedPtr msg)
{
    cv::rectangle(imgFace, cv::Point(msg->x_offset, msg->y_offset),
                  cv::Point(msg->x_offset + msg->width, msg->y_offset + msg->height), cv::Scalar(0, 0, 255), 2,
                  cv::LINE_8);
    cv::imshow(FACE_WIN_NAME, imgFace);
    cv::waitKey(1);
}

void Follow(sensor_msgs::msg::RegionOfInterest::ConstSharedPtr msg)
{
    int img_center = imgFace.cols / 2;
    int face_center = msg->x_offset + msg->width / 2;
    int error = face_center - img_center;

    if (abs(error) < 20) {
        vel_cmd.angular.z = 0.0;
    }

    vel_cmd.angular.z = -Kp * error;

    vel_pub->publish(vel_cmd);
}

void CamRGBCallback(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    imgFace = cv_ptr->image;

    frame_pub->publish(*msg);
}

void FacePoseCallback(sensor_msgs::msg::RegionOfInterest::ConstSharedPtr msg)
{
    DrawRectangle(msg);

    Follow(msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>(NODE_NAME);

    auto rgb_sub = node->create_subscription<sensor_msgs::msg::Image>(CAM_TOPIC_NAME, QOS1, CamRGBCallback);
    frame_pub = node->create_publisher<sensor_msgs::msg::Image>(FRAME_TOPIC_NAME, QOS1);
    vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(VEL_TOPIC_NAME, QOS10);
    auto face_sub =
        node->create_subscription<sensor_msgs::msg::RegionOfInterest>(FACE_TOPIC_NAME, QOS1, FacePoseCallback);

    cv::namedWindow(FACE_WIN_NAME);

    rclcpp::spin(node);

    cv::destroyAllWindows();
    rclcpp::shutdown();
}