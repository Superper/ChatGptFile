#include "SocketBase.h"

std::mutex SocketBase::mtx_;
std::condition_variable SocketBase::cv_;
std::unique_ptr<std::thread> SocketBase::send_thread_;
std::map<int, SocketBase::SocketInfo> SocketBase::clients_;
std::multimap<std::string, std::vector<char>> SocketBase::send_data_;
socket_state SocketBase::socket_state_cb_;

void SocketBase::StartSendThread()
{
    send_thread_ = std::make_unique<std::thread>([=]() {
        while (true)
        {
            std::unique_lock<std::mutex> lock(mtx_);
            cv_.wait(lock); // 等待条件变量

            while (!send_data_.empty())
            {
                auto data = send_data_.begin();

                for (auto client : clients_)
                {
                    if (client.second.client_addr == data->first)
                    {
                        if (client.second.is_connected)
                        {
                            send(client.first, data->second.data(), data->second.size(), 0);
                            break;
                        }
                        else
                        {
                            auto msg = "TCP Client " + client.second.client_addr + " disconnected,failed to send";
                            socket_state_cb_(msg);
                            clients_.erase(client.first);
                            break;
                        }
                    }
                }
                send_data_.erase(send_data_.begin());
            }
        }
    });
    send_thread_->detach();
}

void SocketBase::ReadData(int socket_fd)
{
    char buffer[BUFFER_SIZE] = {0};
    int valread;

    while ((valread = read(socket_fd, buffer, BUFFER_SIZE)) > 0)
    {
        socket_msg_cb_(buffer, valread, topic_id_);
        memset(buffer, 0, BUFFER_SIZE);
    }

    if (valread == 0)
    {
        std::unique_lock<std::mutex> lock(mtx_);
        clients_[socket_fd].is_connected = false;
        auto msg = "TCP socket " + clients_[socket_fd].client_addr + " disconnected";
        socket_state_cb_(msg);
    }
    else if (valread == -1)
    {
        perror("recv");
    }

    close(socket_fd);
}
