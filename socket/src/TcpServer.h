
#ifndef HEADER_TcpServer_H
#define HEADER_TcpServer_H
#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "SocketBase.h"

const int PORT = 12345;       // 服务端监听的端口号
const int MAX_CLIENTS = 5;    // 服务端最大连接数
const int BUFFER_SIZE = 1024; // 缓冲区大小

class TcpServer : public SocketBase
{
  public:
    TcpServer() = default;

    ~TcpServer();

    void MakeSocket(uint16_t band_port, std::string inet_addr) override;

    void handle_client(int client_fd, const sockaddr_in &address);
};

#endif