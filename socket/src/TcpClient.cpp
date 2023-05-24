#include "TcpClient.h"

TcpClient::~TcpClient()
{
    std::cout << "Server shutdown...\n";
}

void TcpClient::MakeSocket(const std::string &topic_id, const socket_cb &cb, const uint16_t &band_port,
                           const std::string &inet_add)
{

    topic_id_ = topic_id;
    socket_msg_cb_ = cb;

    threads_ = std::make_unique<std::thread>([=]() {
        sockaddr_in servaddr;

        while (true)
        {
            // Create socket
            client_fd_ = socket(AF_INET, SOCK_STREAM, 0);
            if (client_fd_ == -1)
            {
                std::cerr << "Cannot create socket.\n";
                return;
            }

            // Set server address structure
            memset(&servaddr, 0, sizeof(servaddr));
            servaddr.sin_family = AF_INET;
            servaddr.sin_port = htons(band_port);
            inet_pton(AF_INET, inet_add.c_str(), &servaddr.sin_addr.s_addr);

            // Connect to server
            if (connect(client_fd_, (sockaddr *)&servaddr, sizeof(servaddr)) < 0)
            {
                std::cerr << "Cannot connect to TCP server" << inet_add << ":" << band_port << std::endl;
                close(client_fd_);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            else
            {
                auto client_addr = inet_add + ":" + std::to_string(band_port);
                {
                    std::unique_lock<std::mutex> lck(mtx_);
                    SocketInfo info{client_addr, true};
                    clients_[client_fd_] = info;
                }
                std::cout << "Connect to TCP server: " << client_addr << std::endl;
                // 接收数据, 一直阻塞
                ReadData(client_fd_);
            }
        }
    });
    threads_->detach();
}
