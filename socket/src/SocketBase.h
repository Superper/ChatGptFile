#ifndef HEADER_SocketBase_H
#define HEADER_SocketBase_H

#include <string>
#include <thread>
#include <vector>

class SocketBase
{
  public:
    SocketBase() = default;

    std::string topic_id_;
    virtual void MakeSocket(uint16_t band_port, std::string inet_addr = "") = 0;
    virtual ~SocketBase() = default;

  protected:
    std::thread *threads_;
};

#endif // HEADER_NAME_H