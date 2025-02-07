#include <QApplication>
#include <QString>
#include <QLabel>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <status_interfaces/msg/system_status.hpp>

using SystemStatus = status_interfaces::msg::SystemStatus;
using namespace std;

class SystemStatusDisplayNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<SystemStatus>::SharedPtr sys_status_subscribe_;
    QLabel *label_;

public:
    QString get_qstr_from_msg(const SystemStatus::SharedPtr msg);
    SystemStatusDisplayNode(const string &node_name);
    ~SystemStatusDisplayNode();
};

QString SystemStatusDisplayNode::get_qstr_from_msg(const SystemStatus::SharedPtr msg)
{
    stringstream show_str;
    show_str << "数据时间：\t" << msg->stamp.sec << "\t\n"
             << "用户名：\t" << msg->host_name << "\ts\n"
             << "CPU使用率：\t" << msg->cpu_percent << "\t%\n"
             << "内存使用率：\t" << msg->memory_percent << "\t%\n"
             << "内存总大小：\t" << msg->memory_total << "\tMB\n"
             << "剩余有效内存量：\t" << msg->memory_available << "\tMB\n"
             << "网络发送量：\t" << msg->net_sent << "\tMB\n"
             << "网络接收量：\t" << msg->net_recv << "\tMB\n";
    return QString::fromStdString(show_str.str());
}

SystemStatusDisplayNode::SystemStatusDisplayNode(const string &node_name) : Node(node_name)
{
    this->sys_status_subscribe_ = this->create_subscription<SystemStatus>("sys_status", 10, [&](const SystemStatus::SharedPtr msg) -> void
                                                                          { label_->setText(get_qstr_from_msg(msg)); });
    this->label_ = new QLabel(this->get_qstr_from_msg(std::make_shared<SystemStatus>()));
    this->label_->show();
};

SystemStatusDisplayNode::~SystemStatusDisplayNode()
{
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    rclcpp::init(argc, argv);
    auto node = make_shared<SystemStatusDisplayNode>("sys_status_display");
    thread spin_thread([&]() -> void
                       { rclcpp::spin(node); });
    spin_thread.detach();
    app.exec();
    rclcpp::shutdown();
    return 0;
}
