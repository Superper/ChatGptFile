#ifndef TCPCLIENT_H
#define TCPCLIENT_H
#include "SocketBase.h"

class TcpClient : public SocketBase
{
  public:
    TcpClient() = default;
    ~TcpClient();

    virtual void MakeSocket(const std::string &topic_id, const socket_cb &cb, const uint16_t &band_port,
                            const std::string &inet_add, const uint16_t &local_band_prot) override;
};

#endif