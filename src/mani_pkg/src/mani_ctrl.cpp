#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

const std::string NODE_NAME = "mani_ctrl_node";
const std::string MANI_TOPIC_NAME = "/wpb_home/mani_ctrl";
const int QOS10 = 10;

class ManiCtrlNode : public rclcpp::Node {
public:
    ManiCtrlNode() : rclcpp::Node(NODE_NAME)
    {
        mani_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(MANI_TOPIC_NAME, QOS10);
    }

    void Operate(sensor_msgs::msg::JointState::SharedPtr& mani_msg)
    {
        mani_pub_->publish(*mani_msg);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mani_pub_;
};

void InitJointState(sensor_msgs::msg::JointState::SharedPtr& mani_msg)
{
    mani_msg->name.resize(2);
    mani_msg->name[0] = "lift";
    mani_msg->name[1] = "gripper";

    mani_msg->position.resize(2);
    mani_msg->position[0] = 0.0;
    mani_msg->position[1] = 0.0;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ManiCtrlNode>();

    sensor_msgs::msg::JointState::SharedPtr mani_msg = std::make_shared<sensor_msgs::msg::JointState>();
    InitJointState(mani_msg);

    rclcpp::Rate loop_rate(0.3);
    while (rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), "Pose1");
        mani_msg->position[0] = 0.0;
        mani_msg->position[1] = 0.01;
        node->Operate(mani_msg);
        loop_rate.sleep();

        RCLCPP_INFO(node->get_logger(), "Pose2");
        mani_msg->position[0] = 1.0;
        mani_msg->position[1] = 0.1;
        node->Operate(mani_msg);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}