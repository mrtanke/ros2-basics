#include <iostream>
#include <thread>                // multithreading
#include <chrono>                // for time functions
#include <functional>            // function bindings
#include "cpp-httplib/httplib.h" // HTTP library

class Download
{
private:
    /* data */
public:
    void download(
        const std::string &host,
        const std::string &path,
        const std::function<void(const std::string &, const std::string &)> &call_back_word_count)
    {
        std::cout << "Thread" << std::this_thread::get_id() << std::endl;
        httplib::Client client(host);
        auto response = client.Get(path);
        if(response && response->status == 200)
        {
            call_back_word_count(path, response->body);
        }
        else
        {
            std::cout << "Download failed from path: " << path << std::endl;
        }
    };

    void start_download(
        const std::string &host,
        const std::string &path,
        const std::function<void(const std::string &, const std::string &)> &call_back_word_count) 
    {
        auto download_fun = std::bind(&Download::download, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        std::thread thread(download_fun, host, path, call_back_word_count);
        thread.detach();
    };
};

int main()
{
    auto d = Download();
    auto word_count = [](const std::string &path, const std::string &result) -> void
    {
        std::cout << "Downloaded from path: " << path << ", length: " << result.length() << result.substr(0, 5) << std::endl;
    };

    d.start_download("http://localhost:8000", "/novel/novel1.txt", word_count);
    d.start_download("http://localhost:8000", "/novel/novel2.txt", word_count);
    d.start_download("http://localhost:8000", "/novel/novel3.txt", word_count);

    std::this_thread::sleep_for(std::chrono::seconds(10));
    return 0;
}