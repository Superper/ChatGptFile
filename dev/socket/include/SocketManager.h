// Description: socket连接管理类，用于管理所有的socket连接

#ifndef SOCKETMANAGER_H
#define SOCKETMANAGER_H

#include "../src/SocketFactory.h"
#include "Consumers.h"
#include <iostream>
#include <map>
#include <memory>

enum class SocketType
{
    TCPServer,
    TCPClient,
    UDP,
    UDPMulticast
};

class SocketConnectionManager
{
  public:
    static SocketConnectionManager *getInstance()
    {
        static SocketConnectionManager instance;
        return &instance;
    }

    bool AddConnection(const std::string topic_name, SocketType soket_type, uint16_t port,
                       std::string inet_addr = "127.0.0.1");
    bool RemoveConnection(const std::string topic_name);
    bool SubscribeTopic(const std::string topic_name, Consumers *user);
    bool UnSubscribeTopic(std::string topic_name);
    static void SendData(const std::string &addr, const char *data, uint32_t size);
    ~SocketConnectionManager();

  private:
    void SocketMsg(const char buffer[], const uint16_t &siz, const std::string &topic_name);
    SocketConnectionManager() = default;
    SocketConnectionManager(const SocketConnectionManager &) = delete;
    SocketConnectionManager &operator=(const SocketConnectionManager &) = delete;

    std::vector<std::unique_ptr<SocketBase>> socket_list_;
    std::unordered_map<std::string, std::vector<Consumers *>> users;

    SocketFactoryBase *socket_factory_ = nullptr;
};

#endif // SOCKETMANAGER_H