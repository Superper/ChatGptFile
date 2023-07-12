#include "TcpClient.h"
#include <fcntl.h>

TcpClient::~TcpClient()
{
    threads_->join();
    std::cout << "Client shutdown...\n";
    close(client_fd_);
}

void TcpClient::MakeSocket(const std::string &topic_id, const socket_cb &cb, const uint16_t &band_port,
                           const std::string &inet_add, const uint16_t &local_band_prot)
{

    topic_id_ = topic_id;
    socket_msg_cb_ = cb;

    threads_ = std::make_unique<std::thread>([=]() {
        sockaddr_in servaddr;

        struct sockaddr_in client_addr;
        memset(&client_addr, 0, sizeof(client_addr));
        client_addr.sin_family = AF_INET;
        client_addr.sin_port = htons(local_band_prot);
        client_addr.sin_addr.s_addr = htonl(INADDR_ANY);

        // Set server address structure
        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(band_port);
        inet_pton(AF_INET, inet_add.c_str(), &servaddr.sin_addr.s_addr);

        int optval = 1;
        setsockopt(client_fd_, SOL_SOCKET, SO_REUSEADDR, static_cast<const void *>(&optval),
                   static_cast<socklen_t>(sizeof(optval))); // 设置端口复用

        // int flags = fcntl(client_fd_, F_GETFL, 0);
        // fcntl(client_fd_, F_SETFL, flags | O_NONBLOCK); // 设置为非阻塞模式

        while (true)
        {
            // Create socket
            client_fd_ = socket(AF_INET, SOCK_STREAM, 0);
            if (client_fd_ == -1)
            {
                std::cerr << "Cannot create socket.\n";
            }

            if (bind(client_fd_, (struct sockaddr *)&client_addr, sizeof(client_addr)) == -1)
            {
                std::cerr << "Error binding client socket to port " << local_band_prot << std::endl;
            }

            // Connect to server
            auto msg = std::string("Trying connect to TCP server ") + inet_add + ":" + std::to_string(band_port);
            socket_state_cb_(msg);
            if (connect(client_fd_, (sockaddr *)&servaddr, sizeof(servaddr)) < 0)
            {
                std::cerr << msg << std::endl;
                close(client_fd_);
            }
            else
            {
                auto client_addr = inet_add + ":" + std::to_string(band_port);
                {
                    std::unique_lock<std::mutex> lck(mtx_);
                    SocketInfo info{client_addr, true};
                    clients_[client_fd_] = info;
                }
                auto msg = "TCP server: " + client_addr + " connected";
                socket_state_cb_(msg);

                // 接收数据, 一直阻塞
                ReadData(client_fd_);
            }
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    });
}
