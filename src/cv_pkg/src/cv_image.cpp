#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

const std::string NODE_NAME = "cv_image_node";
const std::string CAM_TOPIC_NAME = "/kinect2/qhd/image_raw";
const std::string CV_WIN_NAME = "RGB";
const int QOS1 = 1;

std::shared_ptr<rclcpp::Node> node;

void CamRGBCallback(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat imgOriginal = cv_ptr->image;
    cv::imshow("RGB", imgOriginal);
    cv::waitKey(1);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>(NODE_NAME);
    auto rgb_sub = node->create_subscription<sensor_msgs::msg::Image>(CAM_TOPIC_NAME, QOS1, CamRGBCallback);

    cv::namedWindow(CV_WIN_NAME);

    rclcpp::spin(node);

    cv::destroyAllWindows();
    rclcpp::shutdown();

    return 0;
}