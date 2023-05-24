#include "DataProcess.h"

void DataProcess::Process()
{
    while (true)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this] { return !data_.empty(); });

        while (!data_.empty())
        {
            std::vector<char> data = data_.front();
            data_.pop_back();

            std::cout << "data size - " << data.size() << std::endl;
            for (const auto &i : data)
            {
                std::cout << i;
            }
            std::cout << std::endl;

            auto header = reinterpret_cast<PkgHeader *>(data.data());
            auto temp = (MsgID)header->id;
            auto img_data = img_data_map_.find((MsgID)header->id);
            if (img_data == img_data_map_.end())
            {
                std::cout << "img_data_map cant find header->id: " << header->id << std::endl;
                continue;
            }
            auto processed_data = img_data->second->DataProcessBase(data);
            SocketConnectionManager::SendData("192.168.43.62:12346", processed_data.data(), processed_data.size());
        }
    }
}

DataProcess::DataProcess(const std::string &ip, const uint16_t &port) : ip_(ip), port_(port)
{
    ImgDataProcessFactory *factory = nullptr;
    factory = new VisibleLighetProcessFactory;
    img_data_map_.insert(std::make_pair(MsgID::Visiblelight, factory->CreateImgDataProcess()));
    delete factory;
    factory = new InfraredProcessFactory;
    img_data_map_.insert(std::make_pair(MsgID::Infrared, factory->CreateImgDataProcess()));
    delete factory;
    factory = new OtherLightProcessFactory;
    img_data_map_.insert(std::make_pair(MsgID::OtherLight, factory->CreateImgDataProcess()));
    delete factory;
    factory = new Y9SARProcessFactory;
    img_data_map_.insert(std::make_pair(MsgID::Y9SAR, factory->CreateImgDataProcess()));
    delete factory;
    factory = new OtherSARProcessFactory;
    img_data_map_.insert(std::make_pair(MsgID::OtherSAR, factory->CreateImgDataProcess()));
    delete factory;

    thread_ = std::make_unique<std::thread>(&DataProcess::Process, this);
    thread_->detach();
}
