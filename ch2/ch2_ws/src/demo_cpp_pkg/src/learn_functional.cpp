#include <iostream>
#include <functional>

using namespace std;

void save_with_free_function(const string &filename)
{
    cout << "自由函数, " << filename.c_str() << endl;
}

class FileSave
{
private:
    /* data */
public:
    FileSave(/* args */);
    ~FileSave();

    void save_with_member_function(const string &filename)
    {
        cout << "成员函数, " << filename.c_str() << endl;
    }
};

FileSave::FileSave(/* args */)
{
}

FileSave::~FileSave()
{
}

int main(int argc, char const *argv[])
{
    FileSave file_save;
    // lambda函数
    auto save_with_lambda_function = [](const string &filename) -> void
    {
        cout << "lambda函数, " << filename << endl;
        return;
    };
    function<void(const string &filename)> save1 = save_with_free_function;
    function<void(const string &filename)> save2 = save_with_lambda_function;

    function<void(const string &filename)> save3 = bind(&FileSave::save_with_member_function, &file_save, placeholders::_1);

    save1("a.txt");
    save2("a.txt");
    save3("a.txt");
    return 0;
}
