#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

const std::string NODE_NAME = "waypoint_navigation_node";
const std::string PUB_TOPIC_NAME = "/waterplus/navi_waypoint";
const std::string RESULT_TOPIC_NAME = "/waterplus/navi_result";
const std::vector<std::string> DEFAULT_WAYPOINTS = {"1", "2", "3", "4", "5", "6"};
const size_t START_IDNEX = 0;
const int QOS10 = 10;

class WaypointNavigation : public rclcpp::Node {
public:
    WaypointNavigation() : rclcpp::Node(NODE_NAME), waypoints_(DEFAULT_WAYPOINTS), current_index_(START_IDNEX)
    {
        navigation_pub_ = this->create_publisher<std_msgs::msg::String>(PUB_TOPIC_NAME, QOS10);
        result_sub_ = this->create_subscription<std_msgs::msg::String>(
            RESULT_TOPIC_NAME, QOS10, std::bind(&WaypointNavigation::ResultCallback, this, std::placeholders::_1));
        timer_ =
            this->create_wall_timer(std::chrono::seconds(1), std::bind(&WaypointNavigation::SendFirstWaypoint, this));
    }

private:
    void SendFirstWaypoint()
    {
        timer_->cancel();
        PublishCurrentWaypoint();
    }

    void PublishCurrentWaypoint()
    {
        std_msgs::msg::String msg;
        msg.data = waypoints_[current_index_];

        RCLCPP_INFO(this->get_logger(), "Sending waypoint %s", msg.data.c_str());
        navigation_pub_->publish(msg);
    }
    void ResultCallback(std_msgs::msg::String::ConstSharedPtr msg) {
        if (msg->data == "navi done") {
            RCLCPP_INFO(this->get_logger(), "Arrived at %s", msg->data.c_str());

            // Move to the next waypoint
            current_index_ = (current_index_ + 1) % waypoints_.size();

            // Sleep for half second
            rclcpp::sleep_for(std::chrono::milliseconds(500));

            PublishCurrentWaypoint();
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr result_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> waypoints_;
    size_t current_index_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<WaypointNavigation>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}