#ifndef DATAPROCESS_H
#define DATAPROCESS_H
#include "Consumers.h"
#include <map>
#include"./DataProcess/ImgDataProcessFactory.h"

class DataProcess : public Consumers
{
  private:
  struct SendAddr
  {
    std::string ip;
    uint16_t port;
  };


  ImgDataProcessFactory *img_data_process_factory_;

  std::unordered_map<MsgID, ImgData *> img_data_map_;

  std::vector<uint8_t> ExtractPackets();

public:
  virtual void Process() override;
  DataProcess(msg_callback mb);
  msg_callback mg_;

  ~DataProcess() = default;
  };

#endif // DATAPROCESS_H