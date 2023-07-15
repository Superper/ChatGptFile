#include "Control.h"
#include <SocketManager.h>

#define UDPBINDPORT 8990
#define UDPSENDIP "192.168.112.54"
#define UDPSENDCMDCBPORT 10000
#define UDPSENDHEALTHIP "192.168.112.54"
#define UDPHEALTHPORT 10001

uint16_t Control::send_seq_ = 1;
Control::Control()
{
    send_data.header.srcId = MYID;
    send_data.header.dstId = SRCID;
    send_data.header.type = 1;
    send_data.header.msgType = 0xF2;
    send_data.header.id = swap_endian16(0x1201);
    send_data.header.sendTime = swap_endian64(GetPkgTime());
    send_data.header.sendSeq = swap_endian16(send_seq_++);
    send_data.header.length = swap_endian32(sizeof(send_data));

    udp_ = new QUdpSocket;
    connect(udp_, &QUdpSocket::readyRead, this, &Control::processPendingDatagrams);
    connect(&send_state_tiemr_, &QTimer::timeout, this, &Control::MakeSendState);
    udp_->bind(UDPBINDPORT);
    send_state_tiemr_.start(1000);
}
Control::~Control()
{
    delete udp_;
}

void Control::processPendingDatagrams()
{
    while (udp_->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize(udp_->pendingDatagramSize());

        QHostAddress sender;
        quint16 senderPort;

        udp_->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

        // 处理接收到的数据
        std::vector<uint8_t> temp(datagram.data(), datagram.data() + datagram.size()); // 使用构造函数复制数据
        DataProcessBase(temp);
    }
}

void Control::MakeSendState()
{
    QueHealthState send_data;

    send_data.header.srcId = MYID;
    send_data.header.dstId = SRCID;
    send_data.header.type = 1;
    send_data.header.msgType = 0xF2;
    send_data.header.id = swap_endian16(0x1201);
    send_data.header.sendTime = swap_endian64(GetPkgTime());
    send_data.header.sendSeq = swap_endian16(send_seq_++);
    send_data.header.length = swap_endian32(sizeof(send_data));

    udp_->writeDatagram((char *)&send_data, sizeof(send_data), QHostAddress(UDPSENDHEALTHIP), UDPHEALTHPORT);
}

std::vector<uint8_t> Control::DataProcessBase(const std::vector<uint8_t> &data)
{
    auto cmd = (UserControl *)data.data();
    SRCID = cmd->header.srcId;

    qDebug() << "recv ctl";
    if (cmd->control_type == 1)
    {
        if (cmd->operator_type == 1) // 启动外部程序
        {
            if_send_ = true;
            mg_("External plug-in started, waiting for data...");
            send_state_tiemr_.start(1000);
        }
        else if (cmd->operator_type == 2) // 关闭外部程序
        {
            if_send_ = false;
            sar_c_ = 0;
            inf_c_ = 0;
            light_c_ = 0;
            mg_("External plug-in stopped,clear target count");
            send_state_tiemr_.stop();
        }
    }
    send_data.control_type = 1;
    send_data.operator_type = cmd->operator_type;
    send_data.reply_type = 1;

    udp_->writeDatagram((char *)&send_data, sizeof(send_data), QHostAddress(UDPSENDIP), UDPSENDCMDCBPORT);

    // 回复空数据，防止将其发送给处理软件
    std::vector<uint8_t> temp;
    return temp;
}
