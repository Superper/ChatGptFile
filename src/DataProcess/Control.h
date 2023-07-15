#ifndef CONTROL_HPP
#define CONTROL_HPP
#include "ImgData.h"
#include <QUdpSocket>
#include <cstdlib>
#include <memory>
#include <QTimer>

#define EXTERNAL_PROGRAM "/Users/zhx/Work/ZN/dev/src/DataProcess/test/test"
#define KILL_EXTERNAL_PROGRAM "pkill /Users/zhx/Work/ZN/dev/src/DataProcess/test/test"

class Control : public QUdpSocket, public ImgData
{
    Q_OBJECT
  public:
    Control();

    ~Control();

  protected:
    ControlMsgReply send_data;
    static uint16_t send_seq_;

    QUdpSocket *udp_;
    QTimer send_state_tiemr_;
    void processPendingDatagrams();
    void MakeSendState();
    virtual std::vector<uint8_t> DataProcessBase(const std::vector<uint8_t> &data) override;
};
#endif // CONTROL_HPP