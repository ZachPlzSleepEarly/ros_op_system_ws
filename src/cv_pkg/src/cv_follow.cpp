#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>

const std::string NODE_NAME = "cv_follow_node";
const std::string VEL_TOPIC_NAME = "/cmd_vel";
const std::string CAM_TOPIC_NAME = "/kinect2/qhd/image_raw";
const std::string RGB_WIN_NAME = "RGB";
const std::string RESULT_WIN_NAME = "Result";
const int QOS10 = 10;

static int iLowH = 10;
static int iHighH = 40;

static int iLowS = 90;
static int iHighS = 255;

static int iLowV = 1;
static int iHighV = 255;

std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
geometry_msgs::msg::Twist vel_cmd;

void CamRGBCallback(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat imgOriginal = cv_ptr->image;

    cv::Mat imgHSV;
    cv::cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV);

    std::vector<cv::Mat> hsvSplit;
    cv::split(imgHSV, hsvSplit);
    cv::equalizeHist(hsvSplit[2], hsvSplit[2]);
    cv::merge(hsvSplit, imgHSV);

    cv::Mat imgThreshold;
    cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThreshold);
    
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(imgThreshold, imgThreshold, cv::MORPH_OPEN, element);
    cv::morphologyEx(imgThreshold, imgThreshold, cv::MORPH_CLOSE, element);

    int nTargetX = 0;
    int nTargetY = 0;
    int nPixCount = 0;
    int nImgWidth = imgThreshold.cols;
    int nImgHeight = imgThreshold.rows;
    for (int y = 0; y < nImgHeight; ++y) {
        for (int x = 0; x < nImgWidth; ++x) {
            if (imgThreshold.data[y * nImgWidth + x] == 255) {
                nTargetX += x;
                nTargetY += y;
                nPixCount++;
            }
        }
    }

    if (nPixCount > 0) {
        nTargetX /= nPixCount;
        nTargetY /= nPixCount;
        RCLCPP_INFO(node->get_logger(), "Target (%d, %d)  PixelCount = %d", nTargetX, nTargetY, nPixCount);
        cv::Point line_begin = cv::Point(nTargetX - 10, nTargetY);
        cv::Point line_end = cv::Point(nTargetX + 10, nTargetY);
        cv::line(imgOriginal, line_begin, line_end, cv::Scalar(255, 0, 0), 3);
        line_begin.x = nTargetX;
        line_begin.y = nTargetY - 10;
        line_end.x = nTargetX;
        line_end.y = nTargetY + 10;
        cv::line(imgOriginal, line_begin, line_end, cv::Scalar(255, 0, 0), 3);

        float fVelForward = (float(nImgHeight) / 2 - nTargetY) * 0.002;
        float fVelTurn = (float(nImgWidth) / 2 - nTargetX) * 0.003;
        vel_cmd.linear.x = fVelForward;
        vel_cmd.linear.y = 0;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = fVelTurn;
    } else {
        RCLCPP_INFO(node->get_logger(), "Target disappeared ...");
        vel_cmd.linear.x = 0;
        vel_cmd.linear.y = 0;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = 0;
    }

    vel_pub->publish(vel_cmd);
    
    cv::imshow(RESULT_WIN_NAME, imgThreshold);
    cv::imshow(RGB_WIN_NAME, imgOriginal);
    cv::waitKey(5);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>(NODE_NAME);
    vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(VEL_TOPIC_NAME, QOS10);
    auto rgb_sub = node->create_subscription<sensor_msgs::msg::Image>(CAM_TOPIC_NAME, QOS10, CamRGBCallback);

    cv::namedWindow(RGB_WIN_NAME);
    cv::namedWindow(RESULT_WIN_NAME);

    rclcpp::spin(node);

    cv::destroyAllWindows();
    rclcpp::shutdown();
}