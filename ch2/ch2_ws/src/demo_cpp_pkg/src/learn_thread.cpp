#include <iostream>
#include <thread>
#include <functional>
#include <chrono>
#include <httplib.h>

using namespace std;
class Download
{
private:
    /* data */
public:
    Download();
    ~Download();
    void download(const string &host, const string &path, function<void(const string &, const string &)> callback_word_count);
    void start_download(const string &host, const string &path, function<void(const string &, const string &)> callback_word_count);
};

void Download::start_download(const string &host, const string &path, function<void(const string &, const string &)> callback_word_count)
{
    auto download_fun = bind(&Download::download, this, placeholders::_1, placeholders::_2, placeholders::_3);
    thread thread(download_fun, host, path, callback_word_count);
    thread.detach();
}

void Download::download(const string &host, const string &path, function<void(const string &, const string &)> callback_word_count)
{
    cout << "当前线程 pid 为: " << this_thread::get_id() << endl;
    httplib::Client cli(host);
    auto resp = cli.Get(path);
    if (resp && resp->status == httplib::OK_200)
    {
        callback_word_count(path, resp->body);
    }
}

Download::Download()
{
}

Download::~Download()
{
}

int main(int argc, char const *argv[])
{
    auto d = Download();
    auto word_count = [](const string &path, const string &result) -> void
    {
        cout << "下载完成, " << path << ", 共计:" << result.length() << "->" << result.substr(0, 5) << endl;
    };
    d.start_download("http://0.0.0.0:8000", "/novel1.txt", word_count);
    d.start_download("http://0.0.0.0:8000", "/novel2.txt", word_count);
    d.start_download("http://0.0.0.0:8000", "/novel3.txt", word_count);

    this_thread::sleep_for(chrono::seconds(5));
    return 0;
}
