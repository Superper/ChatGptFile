#ifndef IMGCHECKRESULT_H
#define IMGCHECKRESULT_H
#include "./DataProcess/SystemInterface.h"
#include "Consumers.h"
#include <QUdpSocket>
#include <functional>
using msg_callback = std::function<void(const std::string &msg)>;
class ImgCheckResult : public Consumers
{
  private:
    // ImageDetectionResult send_data_;
    uint16_t send_seq_ = 1;

    uint car_c_ = 0;
    uint plane_c_ = 0;
    uint ship_c_ = 0;

    void CountType(const std::vector<uint8_t> &data);

    msg_callback mg_;

    QUdpSocket *udp_;

  public:
    void SetMsgCB(msg_callback mg)
    {
        mg_ = mg;
    }
    ImgCheckResult();
    ~ImgCheckResult();

    virtual void Process() override;
};

#endif
