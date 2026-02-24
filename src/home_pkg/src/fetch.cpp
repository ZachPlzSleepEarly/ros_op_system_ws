#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class FetchNode : public rclcpp::Node {
public:
    FetchNode() : Node(NODE_NAME), state_(Step::WAIT)
    {
        using std::placeholders::_1;

        navi_pub_ = this->create_publisher<std_msgs::msg::String>(NAVI_TOPIC, QOS10);

        behavior_pub_ = this->create_publisher<std_msgs::msg::String>(BEHAVIOR_TOPIC, QOS10);

        navi_result_sub_ = this->create_subscription<std_msgs::msg::String>(
            NAVI_RESULT_TOPIC, QOS10, std::bind(&FetchNode::NaviResultCallback, this, _1));

        grab_result_sub_ = this->create_subscription<std_msgs::msg::String>(
            GRAB_RESULT_TOPIC, QOS10, std::bind(&FetchNode::GrabResultCallback, this, _1));

        startup_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&FetchNode::Start, this));
    }

private:
    // =======================
    // 常量定义（编译期常量）
    // =======================

    static constexpr const char* NODE_NAME = "fetch_node";

    static constexpr const char* NAVI_TOPIC = "/waterplus/navi_waypoint";
    static constexpr const char* BEHAVIOR_TOPIC = "/wpb_home/behavior";
    static constexpr const char* NAVI_RESULT_TOPIC = "/waterplus/navi_result";
    static constexpr const char* GRAB_RESULT_TOPIC = "/wpb_home/grab_result";

    static constexpr const char* DESTINATION_KITCHEN = "kitchen";
    static constexpr const char* DESTINATION_GUEST = "guest";
    static constexpr const char* NAVI_DONE = "navi done";
    static constexpr const char* GRAB_DONE = "grab done";
    static constexpr const char* START_GRAB = "start grab";

    static constexpr int QOS10 = 10;

    // =======================
    // 状态机定义
    // =======================

    enum class Step { WAIT, GOTO_KITCHEN, GRAB_DRINK, GOTO_GUEST, DONE };

    void Start()
    {
        startup_timer_->cancel();

        if (state_ == Step::WAIT) {
            PublishString(navi_pub_, DESTINATION_KITCHEN);
            state_ = Step::GOTO_KITCHEN;

            RCLCPP_INFO(this->get_logger(), "[STEP_WAIT] -> [STEP_GOTO_KITCHEN]");
        }
    }

    void NaviResultCallback(const std_msgs::msg::String::ConstSharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "[NaviResultCallback] %s", msg->data.c_str());

        if (state_ == Step::GOTO_KITCHEN && msg->data == NAVI_DONE) {
            PublishString(behavior_pub_, START_GRAB);
            state_ = Step::GRAB_DRINK;

            RCLCPP_INFO(this->get_logger(), "[STEP_GOTO_KITCHEN] -> [STEP_GRAB_DRINK]");
        } else if (state_ == Step::GOTO_GUEST && msg->data == NAVI_DONE) {
            state_ = Step::DONE;

            RCLCPP_INFO(this->get_logger(), "[STEP_GOTO_GUEST] -> [STEP_DONE]");
        }
    }

    void GrabResultCallback(const std_msgs::msg::String::ConstSharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "[GrabResultCallback] %s", msg->data.c_str());

        if (state_ == Step::GRAB_DRINK && msg->data == GRAB_DONE) {
            PublishString(navi_pub_, DESTINATION_GUEST);
            state_ = Step::GOTO_GUEST;

            RCLCPP_INFO(this->get_logger(), "[STEP_GRAB_DRINK] -> [STEP_GOTO_GUEST]");
        }
    }

    // =======================
    // 工具函数（避免重复代码）
    // =======================

    void PublishString(const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& pub, const std::string data) const
    {
        std_msgs::msg::String msg;
        msg.data = data;
        pub->publish(msg);
    }

private:
    Step state_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navi_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr behavior_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navi_result_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grab_result_sub_;

    rclcpp::TimerBase::SharedPtr startup_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FetchNode>());
    rclcpp::shutdown();
    return 0;
}