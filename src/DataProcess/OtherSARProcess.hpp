#ifndef OTHERSARPROCESS_HPP
#define OTHERSARPROCESS_HPP
#include "ImgData.h"

class OtherSARProcess : public ImgData
{
  public:
    virtual std::vector<uint8_t> DataProcessBase(const std::vector<uint8_t> &data) override
    {

        auto tmep = (OtherSARMsg1 *)data.data();

        std::cout << "img_name: " << tmep->img_name << std::endl;
        std::cout << "imageSource: " << static_cast<int>(tmep->imageSource) << std::endl;
        std::cout << "workMode: " << static_cast<int>(tmep->workMode) << std::endl;
        std::cout << "imageResolution: " << tmep->imageResolution << std::endl;
        std::cout << "imageRowCount: " << tmep->imageRowCount << std::endl;
        std::cout << "imageColumnCount: " << tmep->imageColumnCount << std::endl;
        std::cout << "quantizationBit: " << static_cast<int>(tmep->quantizationBit) << std::endl;
        std::cout << "imageFeaturePosition.REP: " << tmep->imageFeaturePosition.REP << std::endl;
        std::cout << "imageFeaturePosition.lon1: " << tmep->imageFeaturePosition.lon1 << std::endl;
        std::cout << "imageFeaturePosition.lat1: " << tmep->imageFeaturePosition.lat1 << std::endl;
        std::cout << "imageFeaturePosition.alt1: " << tmep->imageFeaturePosition.alt1 << std::endl;
        std::cout << "imageFeaturePosition.lon2: " << tmep->imageFeaturePosition.lon2 << std::endl;
        std::cout << "imageFeaturePosition.lat2: " << tmep->imageFeaturePosition.lat2 << std::endl;
        std::cout << "imageFeaturePosition.alt2: " << tmep->imageFeaturePosition.alt2 << std::endl;
        std::cout << "imageFeaturePosition.lon3: " << tmep->imageFeaturePosition.lon3 << std::endl;
        std::cout << "imageFeaturePosition.lat3: " << tmep->imageFeaturePosition.lat3 << std::endl;
        std::cout << "imageFeaturePosition.alt3: " << tmep->imageFeaturePosition.alt3 << std::endl;
        std::cout << "totalPacketCount: " << tmep->totalPacketCount << std::endl;
        std::cout << "subPacketNumber: ";
        std::cout << tmep->totalPacketCount;
        std::cout << std::endl;
        std::cout << "validDataLength: " << tmep->validDataLength << std::endl;

        send_msg.resize(data.size() - sizeof(PkgHeader) - 20);
        memcpy(&send_msg, data.data() + sizeof(PkgHeader) + 20, data.size() - sizeof(PkgHeader) - 20);

        if (if_send_)
        {
            mg_("sar-" + std::to_string(++sar_c_));
            return send_msg;
        }
        else
        {
            std::vector<uint8_t> temp;
            return temp;
        }
    }
};

#endif // OTHERSARPROCESS_HPP