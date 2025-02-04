#define HTTPLIB_IMPLEMENTATION
#include <httplib.h>
#include <chrono>
#include <iostream>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std;

class NovelPubNode : public rclcpp::Node
{
private:
    httplib::Client cli_; // 使用组合而不是继承
    // 创建话题发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr novel_publisher_;
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;
    // 创建队列
    queue<string> novel_queue_;
    void timer_callback();

public:
    NovelPubNode(const string &node_name, const string &host, const int port, const int time_gap);
    void download_novel(const string &url);
    ~NovelPubNode();
};

void NovelPubNode::timer_callback()
{
    // 创建消息
    std_msgs::msg::String message;
    if (!this->novel_queue_.empty())
    {
        message.data = this->novel_queue_.front();
        this->novel_queue_.pop();
        // 日志打印
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        // 发布消息
        this->novel_publisher_->publish(message);
    }
}

NovelPubNode::NovelPubNode(const string &node_name, const string &host, const int port, const int time_gap) : rclcpp::Node(node_name), cli_(host, port)
{
    RCLCPP_INFO(this->get_logger(), "NovelPubNode is running...");
    this->novel_publisher_ = this->create_publisher<std_msgs::msg::String>("novel", 10);
    // 创建定时器
    this->timer_ = this->create_wall_timer(chrono::seconds(time_gap), bind(&NovelPubNode::timer_callback, this));
}

void NovelPubNode::download_novel(const string &path)
{
    // 在这里实现下载小说的逻辑
    cout << "当前线程 pid 为: " << this_thread::get_id() << endl;
    auto resp = this->cli_.Get(path);
    if (resp && resp->status == httplib::OK_200)
    {
        RCLCPP_INFO(this->get_logger(), "get %s success, all counts %ld\n", path.c_str(), resp.value().body.length());
        auto novel_content = resp.value().body;
        // 将数据分割塞入队列
        string tmp;
        istringstream iss(novel_content);
        while (getline(iss, tmp))
        {
            this->novel_queue_.push(tmp);
        }
    }
}

NovelPubNode::~NovelPubNode()
{
    // 可以在这里添加清理代码
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NovelPubNode>("novel_pub_node", "0.0.0.0", 8000, 5);
    node->download_novel("/novel1.txt");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
