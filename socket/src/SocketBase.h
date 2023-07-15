#ifndef HEADER_SocketBase_H
#define HEADER_SocketBase_H

#include <arpa/inet.h>
#include <condition_variable>
#include <cstring>
#include <functional>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>
#define BUFFER_SIZE 2048
#define MAX_CLIENTS 5

using socket_cb = std::function<void(const char buffer[], const uint16_t &size, const std::string &topic_name)>;
using socket_state = std::function<void(const std::string &msg)>;

class SocketBase
{
  public:
    SocketBase() = default;

    std::string GetTopicID()
    {
        return topic_id_;
    }

    virtual void MakeSocket(const std::string &topic_id, const socket_cb &cb, const uint16_t &band_port,
                            const std::string &inet_addr, const uint16_t &local_band_prot) = 0;

    virtual ~SocketBase() = default;

    inline static void TcpSendData(const std::string &addr, const char *data, uint32_t size)
    {
        std::unique_lock<std::mutex> lck(mtx_tcp_);
        tcp_send_data_.insert({addr, std::vector<uint8_t>(data, data + size)});
        cv_tcp_.notify_one();
    }
    static void StartSendTCPThread();

    void ReadData(int socket_fd);
    static void SetSocketState(const socket_state &cb)
    {
        socket_state_cb_ = cb;
    }

  protected:
    std::unique_ptr<std::thread> threads_ = nullptr;
    std::vector<std::unique_ptr<std::thread>> client_threads_;
    std::string topic_id_;
    socket_cb socket_msg_cb_;
    static socket_state socket_state_cb_;

    int server_fd_;
    int client_fd_;

    static std::mutex mtx_tcp_;
    static std::condition_variable cv_tcp_;
    static std::unique_ptr<std::thread> tcp_send_thread_;
    static std::multimap<std::string, std::vector<uint8_t>> tcp_send_data_;
    struct SocketInfo
    {
        std::string client_addr;
        bool is_connected;
    };
    static std::map<int, SocketInfo> tcp_clients_;
};
#endif // HEADER_SocketBase_H
