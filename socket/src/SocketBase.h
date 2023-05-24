#ifndef HEADER_SocketBase_H
#define HEADER_SocketBase_H

#include <arpa/inet.h>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>
#define BUFFER_SIZE 2048
#define MAX_CLIENTS 5

using socket_cb = std::function<void(const char buffer[], const uint16_t &size, const std::string &topic_name)>;

class SocketBase
{
  public:
    SocketBase();

    std::string GetTopicID()
    {
        return topic_id_;
    }

    virtual void MakeSocket(const std::string &topic_id, const socket_cb &cb, const uint16_t &band_port,
                            const std::string &inet_addr) = 0;
    virtual ~SocketBase() = default;

    inline static void SendData(const std::string &addr, const char *data, uint32_t size)
    {
        std::unique_lock<std::mutex> lck(mtx_);
        send_data_.insert({addr, std::vector<char>(data, data + size)});
        cv_.notify_one();
    }

    void ReadData(int socket_fd);

  protected:
    std::unique_ptr<std::thread> threads_ = nullptr;
    std::vector<std::unique_ptr<std::thread>> client_threads_;
    std::unique_ptr<std::thread> send_thread_ = nullptr;
    std::string topic_id_;
    socket_cb socket_msg_cb_;

    int server_fd_;
    int client_fd_;

    static std::mutex mtx_;
    static std::condition_variable cv_;
    static std::multimap<std::string, std::vector<char>> send_data_;
    struct SocketInfo
    {
        std::string client_addr;
        bool is_connected;
    };
    static std::map<int, SocketInfo> clients_;
};
