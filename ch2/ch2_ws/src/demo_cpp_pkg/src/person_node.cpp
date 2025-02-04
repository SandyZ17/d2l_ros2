#include "iostream"
#include "rclcpp/rclcpp.hpp"

using namespace std;
class PersonNode : public rclcpp::Node
{
private:
    string name_;
    int age_;

public:
    PersonNode(const string &node_name, const string &name, const int &age);
    void eat(const string &food_name);
    ~PersonNode();
};

void PersonNode::eat(const string &food_name)
{
    RCLCPP_INFO(this->get_logger(), "我是%s, 我今年%d岁, 我爱吃%s", this->name_.c_str(), this->age_, food_name.c_str());
}

PersonNode::PersonNode(const string &node_name, const string &name, const int &age)
    : Node(node_name)
{
    this->age_ = age;
    this->name_ = name;
}

PersonNode::~PersonNode()
{
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PersonNode>("person_node", "张三", 18);
    node->eat("苹果");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}