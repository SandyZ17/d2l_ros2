#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std;
using namespace chrono_literals;

class TurtleControlNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr geometry_twist_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_call_back();

public:
    explicit TurtleControlNode(const string &node_name, const int timer_gap);
    ~TurtleControlNode();
};

void TurtleControlNode::timer_call_back()
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.set__x(1.0);
    msg.angular.set__z(0.5);
    this->geometry_twist_publisher_->publish(msg);
}

TurtleControlNode::TurtleControlNode(const string &node_name, const int timer_gap) : Node(node_name)
{
    this->geometry_twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    timer_ = this->create_wall_timer(chrono::seconds(timer_gap), bind(&TurtleControlNode::timer_call_back, this));
}

TurtleControlNode::~TurtleControlNode()
{
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = make_shared<TurtleControlNode>("turtle_circle_node", 1);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
