#include "TcpServer.h"

TcpServer::~TcpServer()
{
    if (threads_ != nullptr)
    {
        delete threads_;
    }
    std::cout << "Server shutdown...\n";
}

void TcpServer::MakeSocket(uint16_t band_port, std::string inet_addr)
{
    //在新线程中创建套接字，绑定端口，监听客户端连接
    threads_ = new std::thread([=]() {
        int server_fd, client_fd;
        struct sockaddr_in address;

        // 创建套接字，使用 IPv4 协议
        if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
        {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }

        // 设置套接字选项，允许地址重用
        int opt = 1;
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
        {
            perror("setsockopt failed");
            exit(EXIT_FAILURE);
        }

        // 绑定套接字到指定端口
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(PORT);
        if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
        {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }

        // 启动监听，最大等待队列为 MAX_CLIENTS
        if (listen(server_fd, MAX_CLIENTS) < 0)
        {
            perror("listen failed");
            exit(EXIT_FAILURE);
        }

        std::cout << "Listening on port " << PORT << "...\n";

        // 接受客户端连接，并启动新的线程处理客户端请求
        int addrlen = sizeof(address);
        while (true)
        {
            if ((client_fd = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0)
            {
                perror("accept failed");
                exit(EXIT_FAILURE);
            }
            std::thread t(std::bind(&TcpServer::handle_client,this, client_fd, address));
            t.detach();
        }
    });
}

void TcpServer::handle_client(int client_fd, const sockaddr_in &address)
{
    char buffer[BUFFER_SIZE] = {0};
    int valread;

    // 打印连接成功信息
    char ip_str[INET_ADDRSTRLEN];
    if (inet_ntop(AF_INET, &address.sin_addr, ip_str, INET_ADDRSTRLEN) == nullptr)
    {
        perror("inet_ntop failed");
        exit(EXIT_FAILURE);
    }
    std::cout << "Connection established with " << ip_str << ":" << ntohs(address.sin_port) << "...\n";

    // 读取客户端发送的数据，并回显给客户端
    while ((valread = read(client_fd, buffer, BUFFER_SIZE)) > 0)
    {
        std::cout << "Received data from client: " << buffer << "\n";
        send(client_fd, buffer, valread, 0);
        memset(buffer, 0, BUFFER_SIZE);
    }

    // 关闭客户端套接字
    close(client_fd);
    std::cout << "Connection closed with " << ip_str << ":" << ntohs(address.sin_port) << "...\n";
}