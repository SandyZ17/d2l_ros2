#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <hwinfo/hwinfo.h>
#include <hwinfo/utils/unit.h>
#include <chrono>
#include "status_interfaces/msg/system_status.hpp"

using namespace std;
using hwinfo::unit::bytes_to_MiB;

class SysStatusPubNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<status_interfaces::msg::SystemStatus>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_call_back_();

public:
    explicit SysStatusPubNode(const string &, const int);
    ~SysStatusPubNode();
};

void SysStatusPubNode::timer_call_back_()
{
    auto msg = status_interfaces::msg::SystemStatus();
    // 获取hostname
    char hostname[256];
    gethostname(hostname, sizeof(hostname));
    msg.set__host_name(string(hostname));

    // 获取cpu使用
    auto cpu = hwinfo::getAllCPUs();

    // 获取mem使用率
    hwinfo::Memory mem;
    msg.set__memory_total(bytes_to_MiB(mem.total_Bytes()));
    msg.set__memory_available(bytes_to_MiB(mem.available_Bytes()));
    // bug修复：内存使用率计算
    msg.set__memory_percent((mem.total_Bytes() - mem.available_Bytes()) * 100.0 / mem.total_Bytes());

    // 获取当前时间戳
    // bug修复：时间戳获取
    rclcpp::Time ros_time_now = this->now();
    msg.stamp.set__nanosec(ros_time_now.nanoseconds());
    msg.stamp.set__sec(ros_time_now.seconds());
    RCLCPP_INFO(this->get_logger(), "send msg: hostname %s, cpu total %f, available %f", msg.host_name.c_str(), msg.memory_total, msg.memory_available);
    // 发送相关消息
    status_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "send finish");
}

SysStatusPubNode::SysStatusPubNode(const string &node_name, const int timer_gap) : Node(node_name)
{
    this->status_publisher_ = this->create_publisher<status_interfaces::msg::SystemStatus>("sys_status", 10);
    this->timer_ = this->create_wall_timer(chrono::seconds(timer_gap), bind(&SysStatusPubNode::timer_call_back_, this));
}

SysStatusPubNode::~SysStatusPubNode()
{
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = make_shared<SysStatusPubNode>("sys_status_pub", 1);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
