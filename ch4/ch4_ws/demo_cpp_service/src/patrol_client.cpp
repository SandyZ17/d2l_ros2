#include <iostream>
#include <ctime>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include "chapter_interfaces/srv/patrol.hpp"

using namespace std;
using Patrol = chapter_interfaces::srv::Patrol;
using SetParam = rcl_interfaces::srv::SetParameters;

class PatrolClient : public rclcpp::Node
{
private:
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_call_back();

public:
    PatrolClient(const string &);
    shared_ptr<SetParam::Response> call_set_parameters(rcl_interfaces::msg::Parameter &param);
    void update_server_param_k(double k);
    ~PatrolClient();
};

void PatrolClient::timer_call_back()
{
    while (!this->patrol_client_->wait_for_service(chrono::seconds(1)))
    {
        // 等待服务端上线
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线...");
    }
    auto req = std::make_shared<Patrol::Request>();
    req->set__target_x(rand() % 15);
    req->set__target_y(rand() % 15);
    RCLCPP_INFO(this->get_logger(), "请求巡逻(%f,%f)", req->target_x, req->target_y);
    this->patrol_client_->async_send_request(
        req,
        [&](rclcpp::Client<Patrol>::SharedFuture result_future) -> void
        {
            auto resp = result_future.get();
            if (resp->result == Patrol::Response::SUCCESS)
            {
                RCLCPP_INFO(this->get_logger(), "目标点处理成功");
            }
            else if (resp->result == Patrol::Response::FAIL)
            {
                RCLCPP_INFO(this->get_logger(), "目标点处理失败");
            }
        });
}

PatrolClient::PatrolClient(const string &node_name) : Node(node_name)
{
    this->patrol_client_ = this->create_client<Patrol>("patrol");
    this->timer_ = this->create_wall_timer(chrono::seconds(10), bind(&PatrolClient::timer_call_back, this));
    srand(time(NULL));
}

shared_ptr<SetParam::Response> PatrolClient::call_set_parameters(rcl_interfaces::msg::Parameter &param)
{
    auto param_client = this->create_client<SetParam>("/turtle_controller/set_parameters");
    while (!param_client->wait_for_service(chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return nullptr;
        }
        RCLCPP_INFO(this->get_logger(), "等待参数设置服务端上线...");
    }
    // 创建请求对象
    auto req = std::make_shared<SetParam::Request>();
    req->parameters.push_back(param);

    auto future = param_client->async_send_request(req);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    return future.get();
}

void PatrolClient::update_server_param_k(double k)
{
    auto param = rcl_interfaces::msg::Parameter();
    // 设置参数名与值
    param.name = "k";
    auto param_value = rcl_interfaces::msg::ParameterValue();
    param_value.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);
    param_value.set__double_value(k);
    param.set__value(param_value);
    // 请求更新参数并处理
    auto resp = call_set_parameters(param);
    if (resp == nullptr)
    {
        RCLCPP_WARN(this->get_logger(), "参数修改失败");
    }
    else
    {
        for (auto &&r : resp->results)
        {
            if (r.successful)
            {
                RCLCPP_INFO(this->get_logger(), "参数 k 已修改为 %f", k);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "参数 k 修改失败, 原因为: %s", r.reason.c_str());
            }
        }
    }
}

PatrolClient::~PatrolClient()
{
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = make_shared<PatrolClient>("patrol_client");
    node->update_server_param_k(1.5);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
