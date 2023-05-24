#ifndef Y9SARPROCESS_HPP
#define Y9SARPROCESS_HPP
#include "ImgData.h"
class Y9SARProcess : public ImgData
{
    public:
    Y9SARProcess(/* args */) = default;
    ~Y9SARProcess() = default;

    virtual std::vector<char> DataProcessBase(const std::vector<char> &data) override
    {
        auto y9sar = (Y9SARMsg1 *)data.data();

        TargDetectionImgMsg1 send_data;
        send_data.data_type = 3; // SAR
        // 图像名称
        send_data.image_source = 2;
        send_data.work_mode = 0;

        send_data.image_resolution = y9sar->resolution;
        send_data.image_rows = y9sar->imageHeight;
        send_data.image_cols = y9sar->imageWidth;

        send_data.imageFeaturePosition.REP = y9sar->imageFeaturePosition.REP;
        send_data.imageFeaturePosition.lon1 =y9sar->imageFeaturePosition.lon1;
        send_data.imageFeaturePosition.lat1 =y9sar->imageFeaturePosition.lat1;
        send_data.imageFeaturePosition.alt1 =y9sar->imageFeaturePosition.alt1;
        send_data.imageFeaturePosition.lon2 =y9sar->imageFeaturePosition.lon2;
        send_data.imageFeaturePosition.lat2 =y9sar->imageFeaturePosition.lat2;
        send_data.imageFeaturePosition.alt2 =y9sar->imageFeaturePosition.alt2;
        send_data.imageFeaturePosition.lon3 =y9sar->imageFeaturePosition.lon3;
        send_data.imageFeaturePosition.lat3 =y9sar->imageFeaturePosition.lat3;
        send_data.imageFeaturePosition.alt3 =y9sar->imageFeaturePosition.alt3;

        auto y9sar2 = (Y9SARMsg2 *)(data.data() + sizeof(Y9SARMsg1) + swap_endian32(y9sar->imageSize));
        send_data.quantization_bits = y9sar2->quantizationBits;

        send_data.total_package_num = y9sar->packetCount; // 总包数
        // todo 子包序号问题
        send_data.valid_data_length = y9sar->imageSize; // 有效数据长度

        std::vector<char> img_msg(data.data() + sizeof(Y9SARMsg1),
                                  data.data() + sizeof(Y9SARMsg1) + swap_endian32(y9sar->imageSize));

        send_data.message_length = swap_endian32(sizeof(send_data) + img_msg.size() + sizeof(PkgTail));

        PkgTail tail;
        send_msg.resize(sizeof(send_data) + img_msg.size() + sizeof(PkgTail));
        memcpy(send_msg.data(), &send_data, sizeof(send_data));
        memcpy(send_msg.data() + sizeof(send_data), img_msg.data(), img_msg.size());
        memcpy(send_msg.data() + sizeof(send_data) + img_msg.size(), &tail, sizeof(PkgTail));

        return send_msg;
    }
};

#endif