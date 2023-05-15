
#ifndef HEADER_SocketFactoryBase_H
#define HEADER_SocketFactoryBase_H
#include "SocketBase.h"
#include "TcpServer.cpp"
class SocketFactoryBase
{
public:
    SocketFactoryBase() = default;
    virtual SocketBase* CreateSocket() = 0;
    virtual ~SocketFactoryBase() = default;
};

class TcpServerFectory : public SocketFactoryBase
{
    virtual SocketBase *CreateSocket() override
    {
        return new TcpServer;
    }
};
#endif