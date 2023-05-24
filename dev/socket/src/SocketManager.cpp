#include "../include/SocketManager.h"

bool SocketConnectionManager::AddConnection(std::string topic_name, SocketType socket_type, uint16_t port,
                                            std::string inet_addr)
{
    for (const auto &socket : socket_list_)
    {
        if (socket->GetTopicID() == topic_name)
        {
            std::cout << "topic_name is already exist" << std::endl;
            return false;
        }
    }

    switch (socket_type)
    {
    case SocketType::TCPServer:
        socket_factory_ = new TcpServerFectory();
        break;

    case SocketType::TCPClient:
        socket_factory_ = new TcpClientFectory();
        break;
    default:
        return false;
        break;
    }

    auto socket = socket_factory_->CreateSocket();

    socket->MakeSocket(topic_name,
                       std::bind(&SocketConnectionManager::SocketMsg, this, std::placeholders::_1,
                                 std::placeholders::_2, std::placeholders::_3),
                       port, inet_addr);

    socket_list_.push_back(std::unique_ptr<SocketBase>(socket));
    users.insert({topic_name, std::vector<Consumers *>()});
    return true;
}

bool SocketConnectionManager::RemoveConnection(const std::string topic_name)
{
    return false;
}

bool SocketConnectionManager::SubscribeTopic(std::string topic_name, Consumers *user)
{
    for (auto& topic : users)
    {
        if (topic.first == topic_name)
        {
            topic.second.push_back(user);
            return true;
        }
    }

    return false;
}

bool SocketConnectionManager::UnSubscribeTopic(std::string topic_name)
{
    //
    return false;
}

void SocketConnectionManager::SendData(const std::string &addr, const char *data, uint32_t size)
{
    SocketBase::SendData(addr, data, size);
}
SocketConnectionManager::~SocketConnectionManager()
{
    for(auto& user:users){
        for(auto& u:user.second){
            delete u;
        }
    }
}

void SocketConnectionManager::SocketMsg(const char buffer[], const uint16_t &siz, const std::string &topic_name)
{
    for (const auto &user : users[topic_name])
    {
        user->Consume(buffer, siz);
    }
}