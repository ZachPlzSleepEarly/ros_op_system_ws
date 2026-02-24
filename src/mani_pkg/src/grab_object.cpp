#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <wpr_simulation2/msg/object.hpp>

#define STEP_WAIT 0
#define STEP_ALIGN_OBJ 1
#define STEP_HAND_UP 2
#define STEP_FORWARD 3
#define STEP_GRAB 4
#define STEP_OBJ_UP 5
#define STEP_BACKWARD 6
#define STEP_DONE 7

const std::string NODE_NAME = "grab_object_node";
const std::string BEHAVIOR_TOPIC_NAME = "/wpb_home/behavior";
const std::string VEL_TOPIC_NAME = "/cmd_vel";
const std::string MANI_TOPIC_NAME = "/wpb_home/mani_ctrl";
const std::string OBJECT_TOPIC_NAME = "/wpb_home/objects_3d";
const int QOS10 = 10;

class GrabObjectNode : public rclcpp::Node {
public:
    GrabObjectNode() : rclcpp::Node(NODE_NAME), state_(STEP_WAIT)
    {
        cmd_pub_ = this->create_publisher<std_msgs::msg::String>(BEHAVIOR_TOPIC_NAME, QOS10);
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(VEL_TOPIC_NAME, QOS10);
        mani_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(MANI_TOPIC_NAME, QOS10);
        object_sub_ = this->create_subscription<wpr_simulation2::msg::Object>(
            OBJECT_TOPIC_NAME, QOS10, std::bind(&GrabObjectNode::ObjectCallback, this, std::placeholders::_1));
    }

    void UpdateStateMachine()
    {
        switch (state_) {
        case STEP_WAIT:
            HandleWait();
            break;
        case STEP_ALIGN_OBJ:
            HandleAlignObj();
            break;
        case STEP_HAND_UP:
            HandleHandUp();
            break;
        case STEP_FORWARD:
            HandleStepForward();
            break;
        case STEP_GRAB:
            HandleGrab();
            break;
        case STEP_OBJ_UP:
            HandleObjUp();
            break;
        case STEP_BACKWARD:
            HandleBackward();
            break;
        case STEP_DONE:
            HandleDone();
            break;
        default:
            break;
        }
    }

private:
    void StopBase()
    {
        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = 0;
        vel_msg.linear.y = 0;
        vel_pub_->publish(vel_msg);
    }

    void InitJointStateMsg(sensor_msgs::msg::JointState& mani_msg)
    {
        mani_msg.name.resize(NUM_JOINTS);
        mani_msg.name[0] = MANI_LIFT;
        mani_msg.name[1] = MANI_GRIPPER;
        mani_msg.position.resize(NUM_JOINTS);
    }

    void NodeSleep(int rep)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(rep));
    }

    void HandleWait()
    {
        std_msgs::msg::String start_msg;
        start_msg.data = START_MSG;
        cmd_pub_->publish(start_msg);
    }

    void ChangeStateTo(int state)
    {
        state_ = state;
    }

    void HandleAlignObj()
    {
        float diff_x = centeroid_x_ - ALIGN_X;
        float diff_y = centeroid_y_ - ALIGN_Y;
        geometry_msgs::msg::Twist vel_msg;
        if (abs(diff_x) > 0.02 || abs(diff_y) > 0.01) {
            vel_msg.linear.x = diff_x * KP_ALIGN_X;
            vel_msg.linear.y = diff_y * KP_ALIGN_Y;
        } else {
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            std_msgs::msg::String stop_msg;
            stop_msg.data = STOP_MSG;
            cmd_pub_->publish(stop_msg);
            ChangeStateTo(STEP_HAND_UP);
        }
        RCLCPP_INFO(this->get_logger(), "[STEP_ALIGN_OBJ] vel=(%.2f, %.2f)", vel_msg.linear.x, vel_msg.linear.y);
        vel_pub_->publish(vel_msg);
    }

    void HandleHandUp()
    {
        RCLCPP_INFO(this->get_logger(), "[START_HAND_UP]");
        sensor_msgs::msg::JointState mani_msg;
        InitJointStateMsg(mani_msg);
        mani_msg.position[0] = centeroid_z_;
        mani_msg.position[1] = WIDTH_CLAW_OPEN;
        mani_pub_->publish(mani_msg);
        NodeSleep(DURATION_HAND_UP);
        ChangeStateTo(STEP_FORWARD);
    }

    void HandleStepForward()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP_FORWARRD] centeroid_x=%.2f", centeroid_x_);
        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = 0.1;
        vel_msg.linear.y = 0.0;
        vel_pub_->publish(vel_msg);
        int forward_duration = (centeroid_x_ - GRAB_DISTANCE) * TIME_SCALLING_FACTOR_GRAB;
        NodeSleep(forward_duration);
        ChangeStateTo(STEP_GRAB);
    }

    void HandleGrab()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP_GRAB]");
        StopBase();
        sensor_msgs::msg::JointState mani_msg;
        InitJointStateMsg(mani_msg);
        mani_msg.position[0] = centeroid_z_;
        mani_msg.position[1] = WIDTH_CLAW_GRABBED;
        mani_pub_->publish(mani_msg);
        NodeSleep(DURATION_GRABBING);
        ChangeStateTo(STEP_OBJ_UP);
    }

    void HandleObjUp()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP_OBJ_UP]");
        sensor_msgs::msg::JointState mani_msg;
        InitJointStateMsg(mani_msg);
        mani_msg.position[0] = centeroid_z_ + HEIGH_LIFT_UP;
        mani_msg.position[1] = WIDTH_CLAW_GRABBED;
        mani_pub_->publish(mani_msg);
        NodeSleep(DURATION_GRABBING);

        ChangeStateTo(STEP_BACKWARD);
    }

    void HandleBackward()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP_BACKWARD]");
        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = -0.1;
        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;
        vel_pub_->publish(vel_msg);
        NodeSleep(DURATION_BACKWARD);

        ChangeStateTo(STEP_DONE);
    }

    void HandleDone()
    {
        RCLCPP_INFO(this->get_logger(), "[STEP_DONE]");
        StopBase();
    }

    void ObjectCallback(const wpr_simulation2::msg::Object::ConstSharedPtr& msg)
    {
        if (state_ == STEP_WAIT) {
            ChangeStateTo(STEP_ALIGN_OBJ);
        }
        if (state_ == STEP_ALIGN_OBJ) {
            centeroid_x_ = msg->x[0];
            centeroid_y_ = msg->y[0];
            centeroid_z_ = msg->z[0];
        }
    }

    const std::string START_MSG = "start objects";
    const std::string STOP_MSG = "stop objects";
    const std::string MANI_LIFT = "lift";
    const std::string MANI_GRIPPER = "gripper";
    const float ALIGN_X = 1.0;
    const float ALIGN_Y = 0.0;
    const float WIDTH_CLAW_OPEN = 0.15;
    const float WIDTH_CLAW_GRABBED = 0.07;
    const float HEIGH_LIFT_UP = 0.05;
    const float GRAB_DISTANCE = 0.65;
    const float KP_ALIGN_X = 0.8;
    const float KP_ALIGN_Y = 0.8;
    const int TIME_SCALLING_FACTOR_GRAB = 20000;
    const int DURATION_HAND_UP = 8000;
    const int DURATION_GRABBING = 5000;
    const int DURATION_BACKWARD = 10000;
    const int NUM_JOINTS = 2;

    int state_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mani_pub_;
    rclcpp::Subscription<wpr_simulation2::msg::Object>::SharedPtr object_sub_;
    float centeroid_x_ = 0.0;
    float centeroid_y_ = 0.0;
    float centeroid_z_ = 0.0;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GrabObjectNode>();

    rclcpp::Rate loop_rate(30);

    while (rclcpp::ok()) {
        node->UpdateStateMachine();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
}