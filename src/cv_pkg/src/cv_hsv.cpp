#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>

const std::string NODE_NAME = "cv_hsv_node";
const std::string CAM_TOPIC_NAME = "/kinect2/qhd/image_raw";
const std::string WIN_NAME = "Threshold";
const std::string RGB_WIN_NAME = "RGB";
const std::string HSV_WIN_NAME = "HSV";
const std::string RESULT_WIN_NAME = "Result";

static int iLowH = 10;
static int iHighH = 40;
static int iLowS = 90;
static int iHighS = 255;
static int iLowV = 1;
static int iHighV = 255;
const int QOS1 = 1;

std::shared_ptr<rclcpp::Node> node;

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
        RCLCPP_INFO(node->get_logger(), "Target (%d, %d) PixelCount = %d", nTargetX, nTargetY, nPixCount);
        cv::Point line_begin = cv::Point(nTargetX - 10, nTargetY);
        cv::Point line_end = cv::Point(nTargetX + 10, nTargetY);
        cv::line(imgOriginal, line_begin, line_end, cv::Scalar(255, 0, 0));
        line_begin.x = nTargetX;
        line_begin.y = nTargetY - 10;
        line_end.x = nTargetX;
        line_end.y = nTargetY + 10;
        cv::line(imgOriginal, line_begin, line_end, cv::Scalar(255, 0, 0));
    } else {
        std::printf("Target disappeared...\n");
    }

    cv::imshow(RGB_WIN_NAME, imgOriginal);
    cv::imshow(HSV_WIN_NAME, imgHSV);
    cv::imshow(RESULT_WIN_NAME, imgThreshold);
    cv::waitKey(5);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>(NODE_NAME);
    auto rgb_sub = node->create_subscription<sensor_msgs::msg::Image>(CAM_TOPIC_NAME, QOS1, CamRGBCallback);
    cv::namedWindow(WIN_NAME, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("LowH", "Threshold", &iLowH, 179);
    cv::createTrackbar("HighH", "Threshold", &iHighH, 179);

    cv::createTrackbar("LowS", "Threshold", &iLowS, 255);
    cv::createTrackbar("HighS", "Threshold", &iHighS, 255);

    cv::createTrackbar("LowV", "Threshold", &iLowV, 255);
    cv::createTrackbar("HighV", "Threshold", &iHighV, 255);

    cv::namedWindow(RGB_WIN_NAME);
    cv::namedWindow(HSV_WIN_NAME);
    cv::namedWindow(RESULT_WIN_NAME);

    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();

    return 0;
}