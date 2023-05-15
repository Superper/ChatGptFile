// Description: socket连接管理类，用于管理所有的socket连接

#ifndef SOCKETMANAGER_H
#define SOCKETMANAGER_H

#include <iostream>
#include<memory>
#include"../src/SocketBase.h"
#include"../src/SocketFactory.h"

enum class SocketType
{
    TCP,
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

    // 这里实现添加、删除、获取连接的接口
    bool AddConnection(const std::string topic_name,SocketType soket_type, uint16_t port, std::string inet_addr = "");
    bool RemoveConnection(const std::string topic_name);
    bool SubscribeTopic(const std::string topic_name);
    bool UnSubscribeTopic(std::string topic_name);

  private:
    SocketConnectionManager() = default;
    SocketConnectionManager(const SocketConnectionManager &) = delete;            // 禁止拷贝构造函数
    SocketConnectionManager &operator=(const SocketConnectionManager &) = delete; // 禁止赋值运算符

    std::unordered_map<int, SocketConnectionManager> connections_; // 存储连接列表
    std::vector<std::unique_ptr<SocketBase>> socket_list_;
    int nextConnId_ = 0;                                            // 连接的唯一ID生成器
    SocketFactoryBase *socket_factory_ = nullptr;                   // socket工厂类
};

#endif // SOCKETMANAGER_H