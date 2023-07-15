#ifndef Y9SARPROCESS_HPP
#define Y9SARPROCESS_HPP
#include "ImgData.h"
class Y9SARProcess : public ImgData
{
  public:
    Y9SARProcess(/* args */) = default;
    ~Y9SARProcess() = default;

    virtual std::vector<uint8_t> DataProcessBase(const std::vector<uint8_t> &data) override
    {
        auto y9sar = (Y9SARMsg1 *)data.data();

        std::cout << "img_name: " << y9sar->img_name << std::endl;
        std::cout << "sensorActivityNumber: " << y9sar->sensorActivityNumber << std::endl;
        std::cout << "antennaType: " << y9sar->antennaType << std::endl;
        std::cout << "antennaCategory: " << y9sar->antennaCategory << std::endl;
        std::cout << "reserved1: ";
        for (int i = 0; i < sizeof(y9sar->reserved1); i++)
        {
            std::cout << static_cast<int>(y9sar->reserved1[i]) << " ";
        }
        std::cout << std::endl;
        std::cout << "imagingStartTime: " << y9sar->imagingStartTime << std::endl;
        std::cout << "imagingEndTime: " << y9sar->imagingEndTime << std::endl;
        std::cout << "reserved2: ";
        for (int i = 0; i < sizeof(y9sar->reserved2); i++)
        {
            std::cout << static_cast<int>(y9sar->reserved2[i]) << " ";
        }
        std::cout << std::endl;
        std::cout << "dataLength: " << y9sar->dataLength << std::endl;
        std::cout << "reserved3: ";
        for (int i = 0; i < sizeof(y9sar->reserved3); i++)
        {
            std::cout << static_cast<int>(y9sar->reserved3[i]) << " ";
        }
        std::cout << std::endl;
        std::cout << "taskNumber: " << y9sar->taskNumber << std::endl;
        std::cout << "aircraftNumber: " << y9sar->aircraftNumber << std::endl;
        std::cout << "reserved4: ";
        for (int i = 0; i < sizeof(y9sar->reserved4); i++)
        {
            std::cout << static_cast<int>(y9sar->reserved4[i]) << " ";
        }
        std::cout << std::endl;
        std::cout << "bootCount: " << y9sar->bootCount << std::endl;
        std::cout << "reserved5: ";
        for (int i = 0; i < sizeof(y9sar->reserved5); i++)
        {
            std::cout << static_cast<int>(y9sar->reserved5[i]) << " ";
        }
        std::cout << std::endl;
        std::cout << "frameNumber: " << y9sar->frameNumber << std::endl;
        std::cout << "packetCount: " << y9sar->packetCount << std::endl;
        std::cout << "packetNumber: " << y9sar->packetNumber << std::endl;
        std::cout << "reserved6: ";
        for (int i = 0; i < sizeof(y9sar->reserved6); i++)
        {
            std::cout << static_cast<int>(y9sar->reserved6[i]) << " ";
        }
        std::cout << std::endl;
        std::cout << "resolution: " << y9sar->resolution << std::endl;
        std::cout << "reserved7: ";
        for (int i = 0; i < sizeof(y9sar->reserved7); i++)
        {
            std::cout << static_cast<int>(y9sar->reserved7[i]) << " ";
        }
        std::cout << std::endl;
        std::cout << "imageHeight: " << y9sar->imageHeight << std::endl;
        std::cout << "imageWidth: " << y9sar->imageWidth << std::endl;
        std::cout << "azimuthPixelSpacing: " << y9sar->azimuthPixelSpacing << std::endl;
        std::cout << "rangePixelSpacing: " << y9sar->rangePixelSpacing << std::endl;
        std::cout << "imageFeaturePosition.REP: " << y9sar->imageFeaturePosition.REP << std::endl;
        std::cout << "imageFeaturePosition.lon1: " << y9sar->imageFeaturePosition.lon1 << std::endl;
        std::cout << "imageFeaturePosition.lat1: " << y9sar->imageFeaturePosition.lat1 << std::endl;
        std::cout << "imageFeaturePosition.alt1: " << y9sar->imageFeaturePosition.alt1 << std::endl;
        std::cout << "imageFeaturePosition.lon2: " << y9sar->imageFeaturePosition.lon2 << std::endl;
        std::cout << "imageFeaturePosition.lat2: " << y9sar->imageFeaturePosition.lat2 << std::endl;
        std::cout << "imageFeaturePosition.alt2: " << y9sar->imageFeaturePosition.alt2 << std::endl;
        std::cout << "imageFeaturePosition.lon3: " << y9sar->imageFeaturePosition.lon3 << std::endl;
        std::cout << "imageFeaturePosition.lat3: " << y9sar->imageFeaturePosition.lat3 << std::endl;
        std::cout << "imageFeaturePosition.alt3: " << y9sar->imageFeaturePosition.alt3 << std::endl;
        std::cout << "imageSize: " << y9sar->imageSize << std::endl;

        TargDetectionImgMsg1 send_data;
        send_data.data_type = 3; // SAR
        // 图像名称
        send_data.image_source = 2;
        send_data.work_mode = 0;

        send_data.image_resolution = y9sar->resolution;
        send_data.image_rows = y9sar->imageHeight;
        send_data.image_cols = y9sar->imageWidth;

        send_data.imageFeaturePosition.REP = y9sar->imageFeaturePosition.REP;
        send_data.imageFeaturePosition.lon1 = y9sar->imageFeaturePosition.lon1;
        send_data.imageFeaturePosition.lat1 = y9sar->imageFeaturePosition.lat1;
        send_data.imageFeaturePosition.alt1 = y9sar->imageFeaturePosition.alt1;
        send_data.imageFeaturePosition.lon2 = y9sar->imageFeaturePosition.lon2;
        send_data.imageFeaturePosition.lat2 = y9sar->imageFeaturePosition.lat2;
        send_data.imageFeaturePosition.alt2 = y9sar->imageFeaturePosition.alt2;
        send_data.imageFeaturePosition.lon3 = y9sar->imageFeaturePosition.lon3;
        send_data.imageFeaturePosition.lat3 = y9sar->imageFeaturePosition.lat3;
        send_data.imageFeaturePosition.alt3 = y9sar->imageFeaturePosition.alt3;

        auto y9sar2 = (Y9SARMsg2 *)(data.data() + sizeof(Y9SARMsg1) + swap_endian32(y9sar->imageSize));

        std::cout << "quantizationBits: " << y9sar2->quantizationBits << std::endl;
        std::cout << "imageType: " << static_cast<int>(y9sar2->imageType) << std::endl;
        std::cout << "reserved8: " << y9sar2->reserved8 << std::endl;
        std::cout << "packetTail: ";
        for (int i = 0; i < sizeof(y9sar2->packetTail); i++)
        {
            std::cout << static_cast<int>(y9sar2->packetTail[i]) << " ";
        }

        send_data.quantization_bits = y9sar2->quantizationBits;

        send_data.total_package_num = y9sar->packetCount; // 总包数
        // todo 子包序号问题

        if (swap_endian32(y9sar->imageSize) > 256)
        {
            send_data.valid_data_length = swap_endian32(256);
        }
        else
        {
            send_data.valid_data_length = y9sar->imageSize; // 有效数据长度
        }

        std::vector<uint8_t> img_msg(data.data() + sizeof(Y9SARMsg1),
                                     data.data() + sizeof(Y9SARMsg1) + swap_endian32(y9sar->imageSize));

        send_data.message_length = swap_endian32(sizeof(send_data) + img_msg.size() + sizeof(PkgTail));

        PkgTail tail;
        send_msg.resize(sizeof(send_data) + img_msg.size() + sizeof(PkgTail));
        memcpy(send_msg.data(), &send_data, sizeof(send_data));
        memcpy(send_msg.data() + sizeof(send_data), img_msg.data(), img_msg.size());
        memcpy(send_msg.data() + sizeof(send_data) + img_msg.size(), &tail, sizeof(PkgTail));

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

#endif