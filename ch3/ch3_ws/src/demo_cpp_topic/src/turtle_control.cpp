#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>

using namespace std;
using namespace chrono_literals;

class TurtleControlNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle_control_subscribe_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void on_pose_receive_(turtlesim::msg::Pose::SharedPtr pose);
    // 目标位置的x坐标
    double target_x_{3.0};
    // 目标位置的y坐标
    double target_y_{4.0};
    // 控制增益，用于调整速度
    double k_{1.0};
    // 最大速度限制
    double max_speed_{3.0};
    // 距离阈值，用于判断是否接近目标位置
    double dis_gate_{0.1};
    // 角度阈值，用于判断是否接近目标角度
    double angle_gate_{0.2};

public:
    explicit TurtleControlNode(const string &node_name, const double target_x, const double target_y);
    ~TurtleControlNode();
};

void TurtleControlNode::on_pose_receive_(turtlesim::msg::Pose::SharedPtr pose)
{
    auto msg = geometry_msgs::msg::Twist();
    // 获取当前位置
    auto current_x = pose->x;
    auto current_y = pose->y;

    RCLCPP_INFO(this->get_logger(), "position now is: (%f, %f)", current_x, current_y);

    // 计算与目标地点之间的距离差
    auto dis = sqrt(pow(this->target_x_ - current_x, 2) + pow(this->target_y_ - current_y, 2));
    auto angle = atan2(this->target_y_ - current_y, this->target_x_ - current_x) - pose->theta;
    cout << "dis: " << dis << "angle:" << angle << endl;
    if (dis > this->dis_gate_)
    {
        if (angle > this->angle_gate_)
        {
            msg.angular.z = fabs(angle);
        }
        else
        {
            msg.linear.set__x(this->k_ * dis);
        }
    }
    if (msg.linear.x > this->max_speed_)
    {
        msg.linear.set__x(this->max_speed_);
    }
    this->velocity_publisher_->publish(msg);
}

TurtleControlNode::TurtleControlNode(const string &node_name, const double target_x, const double target_y) : Node(node_name)
{
    this->turtle_control_subscribe_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, bind(&TurtleControlNode::on_pose_receive_, this, placeholders::_1));
    this->velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    this->target_x_ = target_x;
    this->target_y_ = target_y;
}

TurtleControlNode::~TurtleControlNode()
{
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = make_shared<TurtleControlNode>("turtle_controller", atof(argv[1]), atof(argv[2]));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
