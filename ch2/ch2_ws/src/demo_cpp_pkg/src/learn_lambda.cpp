#include <iostream>
#include <algorithm>

using namespace std;

int main(int argc, char const *argv[])
{
    auto add = [](int a, int b) -> int
    { return a + b; };
    int sum = add(200, 213);
    auto print_sum = [sum]() -> void
    {
        cout << sum << endl;
    };
    print_sum();
    return 0;
}
