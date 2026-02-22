#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/imgcodecs.hpp>

const std::string NODE_NAME = "cv_image_node";
const int QOS1 = 1;

class CVImageNode : public rclcpp::Node {
public:
    CVImageNode() : rclcpp::Node(NODE_NAME)
    {
        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            CAM_TOPIC_NAME, QOS1, std::bind(&CVImageNode::RGBCallback, this, std::placeholders::_1));
    }

private:
    void RGBCallback(sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat imgOriginal = cv_ptr->image;

        if(!is_screenshot_saved_) {
            bool result = cv::imwrite(IMG_SAVED_PATH, imgOriginal);
            RCLCPP_INFO(this->get_logger(), "Save image %s", result ? "successfully" : "failure");
            is_screenshot_saved_ = true;
        }

        cv::imshow(WIN_NAME, imgOriginal);
        cv::waitKey(1);
    }

    const std::string CAM_TOPIC_NAME = "/kinect2/qhd/image_raw";
    const std::string IMG_SAVED_PATH = "/home/zach-ubuntu//Desktop/img.jpg";
    const std::string WIN_NAME = "RGB";

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    bool is_screenshot_saved_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CVImageNode>();
    rclcpp::spin(node);
    
    cv::destroyAllWindows();
    rclcpp::shutdown();

    return 0;
}