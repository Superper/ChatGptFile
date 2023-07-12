
#ifndef HEADER_TcpServer_H
#define HEADER_TcpServer_H


#include "SocketBase.h"


class TcpServer : public SocketBase
{
  public:
    ~TcpServer();

    virtual void MakeSocket(const std::string &topic_id, const socket_cb &cb, const uint16_t &band_port,
                            const std::string &inet_addr, const uint16_t &local_band_prot) override;
};

#endif