#include "DataProcess.h"
#include <SocketManager.h>
#include <thread>
std::vector<uint8_t> DataProcess::ExtractPackets()
{
    std::vector<uint8_t> temp;
    std::vector<uint8_t> head = {0x55, 0x55, 0x55, 0x55};
    std::vector<uint8_t> tail = {0xaa, 0xaa, 0xaa, 0xaa};

    auto it = std::search(data_.begin(), data_.end(), head.begin(), head.end());
    auto itt = std::search(data_.begin(), data_.end(), tail.begin(), tail.end());
    data_.erase(data_.begin(), it);//删除包头之前的数据

    if (it != data_.end() && itt != data_.end()) // 找到包头
    {
        std::uint32_t value;
        if (it + sizeof(value) + head.size() < data_.end()) // 可以取出正确长度
        {
            std::copy(it + head.size(), it + head.size() + sizeof(value) + 1, reinterpret_cast<char *>(&value));
            value = swap_endian32(value);

            if (it + value - 1 < data_.end()) //  拥有完整长度
            {
                auto aa = std::search(it + value - tail.size(), it + value, tail.begin(), tail.end());
                auto b = it + value - tail.size();
                if (aa == b) // 找到包尾
                {
                    temp.insert(temp.begin(), it, it + value);
                    data_.erase(data_.begin(), it + value);
                    return temp;
                }
            }
        }
    }
    return temp;
}
void DataProcess::Process()
{
    while (true)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock);

        while (!data_.empty())
        {
            // 查找包头
            auto data = ExtractPackets();
            if (data.empty())
            {
                break;
            }
            // std::cout << "Receive data size - " << data.size() << std::endl;
            // std::cout << "Receive buffer size - " << data_.size() << std::endl;

            auto header = reinterpret_cast<PkgHeader *>(data.data());
            auto temp = swap_endian16(header->id);
            auto img_data = img_data_map_.find((MsgID)temp);
            if (img_data == img_data_map_.end())
            {
                std::cout << "img_data_map cant find header->id: " << header->id << std::endl;
                continue;
            }

            auto processed_data = img_data->second->DataProcessBase(data);

            for (auto &i : send_addr_vec_)
            {
                Socket::SocketConnectionManager::TcpSendData(i.ip + ":" + std::to_string(i.port),
                                                             (char *)processed_data.data(), processed_data.size());
            }
        }
    }
}

DataProcess::DataProcess(msg_callback mb) : mg_(mb)
{
    ImgDataProcessFactory *factory = nullptr;
    factory = new VisibleLighetProcessFactory;

    auto dp = factory->CreateImgDataProcess();
    dp->SetMsgCB(mb);
    img_data_map_.insert(std::make_pair(MsgID::Visiblelight, dp));
    delete factory;

    factory = new InfraredProcessFactory;
    dp = factory->CreateImgDataProcess();
    img_data_map_.insert(std::make_pair(MsgID::Infrared, dp));
    delete factory;

    factory = new OtherLightProcessFactory;
    dp = factory->CreateImgDataProcess();
    dp->SetMsgCB(mb);
    img_data_map_.insert(std::make_pair(MsgID::OtherLight, dp));
    delete factory;

    factory = new Y9SARProcessFactory;
    dp = factory->CreateImgDataProcess();
    dp->SetMsgCB(mb);
    img_data_map_.insert(std::make_pair(MsgID::Y9SAR, dp));
    delete factory;

    factory = new OtherSARProcessFactory;
    dp = factory->CreateImgDataProcess();
    dp->SetMsgCB(mb);
    img_data_map_.insert(std::make_pair(MsgID::OtherSAR, dp));
    delete factory;

    factory = new ControlFactory;
    dp = factory->CreateImgDataProcess();
    dp->SetMsgCB(mb);
    img_data_map_.insert(std::make_pair(MsgID::ControlCmd, dp));
    delete factory;

    thread_ = std::make_unique<std::thread>(&DataProcess::Process, this);
    thread_->detach();
}