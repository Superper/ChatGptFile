#ifndef DATAPROCESS_H
#define DATAPROCESS_H
#include "Consumers.h"
#include <map>
#include<SocketManager.h>
#include"./DataProcess/ImgDataProcessFactory.h"

class DataProcess : public Consumers
{
  private:
    std::string ip_;
    uint16_t port_;

    ImgDataProcessFactory *img_data_process_factory_;

    std::unordered_map<MsgID,ImgData*> img_data_map_;

  public:
    virtual void Process() override;
    DataProcess(const std::string &ip, const uint16_t &port);
    ~DataProcess() = default;
};

#endif // DATAPROCESS_H