#include "../include/SocketManager.h"
bool SocketConnectionManager::AddConnection(std::string topic_name, SocketType socket_type, uint16_t port,
                                            std::string inet_addr)
{
    for (const auto &socket : socket_list_)
    {
        if (socket->topic_id_ == topic_name)
        {
            std::cout << "topic_name is already exist" << std::endl;
            return false;
        }
    }

    switch (socket_type)
    {
    case SocketType::TCP:
        socket_factory_ = new TcpServerFectory();
        break;

    default:
        return false;
        break;
    }

    auto socket = socket_factory_->CreateSocket();
    socket->MakeSocket(port, inet_addr);
    socket_list_.push_back(std::unique_ptr<SocketBase>(socket));

    return true;
}

bool SocketConnectionManager::RemoveConnection(const std::string topic_name)
{
    return false;
}

bool SocketConnectionManager::SubscribeTopic(std::string topic_name)
{
    return false;
}

bool SocketConnectionManager::UnSubscribeTopic(std::string topic_name)
{
    return false;
}
