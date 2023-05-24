#include "TcpServer.h"

TcpServer::~TcpServer()
{
    std::cout << "Server shutdown...\n";
}

void TcpServer::MakeSocket(const std::string &topic_id, const socket_cb &cb, const uint16_t &band_port,
                           const std::string &ip)
{
    topic_id_ = topic_id;
    socket_msg_cb_ = cb;

    // 在新线程中创建套接字，绑定端口，监听客户端连接
    threads_ = std::make_unique<std::thread>([=]() {
        struct sockaddr_in address;

        // 创建套接字，使用 IPv4 协议
        if ((server_fd_ = socket(AF_INET, SOCK_STREAM, 0)) == 0)
        {
            perror("socket failed");
            return;
            // exit(EXIT_FAILURE);
        }

        // 设置套接字选项，允许地址重用
        int opt = 1;
        if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)))
        {
            perror("setsockopt failed");
            return;
            // exit(EXIT_FAILURE);
        }

        // 绑定套接字到指定端口
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = inet_addr(ip.c_str());
        address.sin_port = htons(band_port);
        if (bind(server_fd_, (struct sockaddr *)&address, sizeof(address)) < 0)
        {
            perror("bind failed");
            return;

            // exit(EXIT_FAILURE);
        }

        // 启动监听，最大等待队列为 MAX_CLIENTS
        if (listen(server_fd_, MAX_CLIENTS) < 0)
        {
            perror("listen failed");
            return;
            // exit(EXIT_FAILURE);
        }

        std::cout << "Tcp Server Listening on " << ip << ":" << band_port << "...\n";
        // 接受客户端连接，并启动新的线程处理客户端请求
        int addrlen = sizeof(address);
        while ((client_fd_ = accept(server_fd_, (struct sockaddr *)&address, (socklen_t *)&addrlen)))
        {

            struct sockaddr_in client_addr;
            socklen_t client_addr_len = sizeof(client_addr);
            if (getpeername(client_fd_, (struct sockaddr *)&client_addr, &client_addr_len) == -1)
            {
                perror("getpeername");
                close(client_fd_);
                return;
            }
            else
            {
                // 将网络字节序转换为点分十进制 IP 地址
                char client_ip[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);

                // 将网络字节序转换为主机字节序
                int client_port = ntohs(client_addr.sin_port);

                std::cout << "TCP Client " << client_ip << ":" << client_port << " connected" << std::endl;
                std::string client_addr = client_ip + std::string(":") + std::to_string(client_port);
                std::unique_lock<std::mutex> lck(mtx_);
                SocketInfo socket_info{client_addr, true};
                clients_[client_fd_] = socket_info;

                // 新建线程用于接收和处理客户端发送的消息
                auto thread = std::make_unique<std::thread>([=]() { ReadData(client_fd_); });
                thread.get()->detach();
                client_threads_.push_back(std::move(thread));
            }
        }
    });
    threads_->detach();
}
