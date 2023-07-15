#include "ImgCheckResult.h"
#include "DataProcess/SystemInterface.h"
#include <SocketManager.h>

ImgCheckResult::ImgCheckResult()
{
    // send_data_.header.srcId = MYID;
    // send_data_.header.dstId = SRCID;
    // send_data_.header.type = 1;
    // send_data_.header.msgType = 0xF2;
    // send_data_.header.id = 0x1002;
    // send_data_.header.sendTime = GetPkgTime();
    // send_data_.header.sendSeq = send_seq_++;
    // send_data_.header.length = sizeof(send_data_);
    thread_ = std::make_unique<std::thread>(&ImgCheckResult::Process, this);
}

ImgCheckResult::~ImgCheckResult()
{
    thread_->join();
}

void ImgCheckResult::Process()
{
    if (udp_ == nullptr)
    {
        udp_ = new QUdpSocket;
    }
    while (true)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this] { return !data_.empty(); });

        while (!data_.empty())
        {
            std::vector<uint8_t> data ;
            // std::vector<uint8_t> data = data_.front();
            data_.pop_back();

            std::cout << "data size - " << data.size() << std::endl;
            for (const auto &i : data)
            {
                std::cout << i;
            }
            std::cout << std::endl;

            CountType(data);

            for (auto &i : send_addr_vec_)
            {
                Socket::SocketConnectionManager::TcpSendData(i.ip + ":" + std::to_string(i.port), (char*)data.data(),
                                                             data.size());
            }

            udp_->writeDatagram((char *)data.data(), data.size(), QHostAddress("127.0.0.1"), 12345);
        }
    }
}

void ImgCheckResult::CountType(const std::vector<uint8_t> &data)
{
    auto cmd = (ImageDetectionResult *)data.data();

    if (cmd->targetType == 0x11 || cmd->targetType == 0x11 || cmd->targetType == 0x13)
    {
        mg_("car-" + std::to_string(++car_c_));
    }
    if (cmd->targetType == 0x21 || cmd->targetType == 0x22 || cmd->targetType == 0x23 || cmd->targetType == 0x24)
    {
        mg_("pla-" + std::to_string(++plane_c_));
    }
    if (cmd->targetType == 0x31 || cmd->targetType == 0x32 || cmd->targetType == 0x33 || cmd->targetType == 0x34 ||
        cmd->targetType == 0x35)
    {
        mg_("shi-" + std::to_string(++ship_c_));
    }
}