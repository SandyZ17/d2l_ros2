#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <chapter_interfaces/srv/patrol.hpp>
#include <rcl_interfaces/msg/parameter.hpp>

using namespace std;
using Patrol = chapter_interfaces::srv::Patrol;
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
    double target_x_{1.0};
    // 目标位置的y坐标
    double target_y_{1.0};
    // 控制增益，用于调整速度
    double k_{1.0};
    // 最大速度限制
    double max_speed_{3.0};
    // 距离阈值，用于判断是否接近目标位置
    double dis_gate_{0.1};
    // 角度阈值，用于判断是否接近目标角度
    double angle_gate_{0.2};
    rclcpp::Service<Patrol>::SharedPtr patro_server_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

public:
    explicit TurtleControlNode(const string &node_name);
    ~TurtleControlNode();
};

void TurtleControlNode::on_pose_receive_(turtlesim::msg::Pose::SharedPtr pose)
{
    auto msg = geometry_msgs::msg::Twist();
    // 获取当前位置
    auto current_x = pose->x;
    auto current_y = pose->y;

    // RCLCPP_INFO(this->get_logger(), "position now is: (%f, %f)", current_x, current_y);

    // 计算与目标地点之间的距离差
    auto dis = sqrt(pow(this->target_x_ - current_x, 2) + pow(this->target_y_ - current_y, 2));
    auto angle = atan2(this->target_y_ - current_y, this->target_x_ - current_x) - pose->theta;

    if (dis > this->dis_gate_)
    {
        if (fabs(angle) > this->angle_gate_)
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

TurtleControlNode::TurtleControlNode(const string &node_name) : Node(node_name)
{
    // 声明参数
    this->declare_parameter("k", 1.0);
    this->declare_parameter("max_speed", 1.0);
    this->get_parameter("max_speed", this->max_speed_);
    this->get_parameter("k", this->k_);

    // 设置参数订阅回调
    this->param_callback_handle_ = this->add_on_set_parameters_callback(
        [&](const std::vector<rclcpp::Parameter> &param) -> rcl_interfaces::msg::SetParametersResult
        {
            // 遍历参数
            for (auto p : param)
            {
                RCLCPP_INFO(this->get_logger(), "更新参数%s, 值为: %f", p.get_name().c_str(), p.as_double());
                if (p.get_name() == "k")
                {
                    this->k_ = p.as_double();
                }
                else if (p.get_name() == "max_speed")
                {
                    this->max_speed_ = p.as_double();
                }
            }
            auto result = rcl_interfaces::msg::SetParametersResult();
            result.set__successful(true);
            return result;
        });

    this->turtle_control_subscribe_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, bind(&TurtleControlNode::on_pose_receive_, this, placeholders::_1));
    this->velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    this->patro_server_ = this->create_service<Patrol>(
        "patrol",
        [&](const std::shared_ptr<Patrol::Request> req, shared_ptr<Patrol::Response> resp) -> void
        {
            if ((0 < req->target_x && req->target_x < 12.0f) && (0 < req->target_y && req->target_y < 12.0f))
            {
                this->target_x_ = req->target_x;
                this->target_y_ = req->target_y;
                resp->result = Patrol::Response::SUCCESS;
            }
            else
            {
                resp->result = Patrol::Response::FAIL;
            }
        });
}

TurtleControlNode::~TurtleControlNode()
{
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = make_shared<TurtleControlNode>("turtle_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
