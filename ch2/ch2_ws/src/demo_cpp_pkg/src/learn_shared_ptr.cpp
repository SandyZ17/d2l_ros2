#include <iostream>
#include <memory>

using namespace std;

int main(int argc, char const *argv[])
{
    // std::make_shared<type>(value)
    auto p1 = make_shared<string>("This is a str.");
    cout << "p1的引用计数为:" << p1.use_count() << ",内存地址为:" << p1.get() << endl;

    auto p2 = p1;
    cout << "p1的引用计数为:" << p1.use_count() << ",内存地址为:" << p1.get() << endl;
    cout << "p2的引用计数为:" << p2.use_count() << ",内存地址为:" << p2.get() << endl;

    // 释放p1指针
    p1.reset();
    cout << "p1的引用计数为:" << p1.use_count() << ",内存地址为:" << p1.get() << endl;
    cout << "p2的引用计数为:" << p2.use_count() << ",内存地址为:" << p2.get() << endl;

    cout << "p2地址指向数据为:" << p2->c_str() << endl;

    return 0;
}
