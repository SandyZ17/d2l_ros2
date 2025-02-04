#include <iostream>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std;

class NovelSubNode : public rclcpp::Node
{
private:
    // 创建话题订阅者
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr novel_subscribe_;
    // 订阅回调函数
    void novel_callback(const std_msgs::msg::String::SharedPtr msg);

public:
    NovelSubNode(const string &node_name);
    ~NovelSubNode();
};

NovelSubNode::NovelSubNode(const string &node_name) : rclcpp::Node(node_name)
{
    // 创建订阅者
    this->novel_subscribe_ = this->create_subscription<std_msgs::msg::String>("novel", 10, bind(&NovelSubNode::novel_callback, this, placeholders::_1));
}

void NovelSubNode::novel_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "订阅小说已更新，当前段落 %s\n", msg->data.c_str());
}
NovelSubNode ::~NovelSubNode()
{
}

int main(int argc, char const *argv[])
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<NovelSubNode>("novel_sub_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
