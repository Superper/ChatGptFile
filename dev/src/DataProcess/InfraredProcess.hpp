#ifndef INFRAEDPROCESS_HPP
#define INFRAEDPROCESS_HPP
#include "ImgData.h"

class InfraredProcess : public ImgData
{
    public:
    InfraredProcess() = default;
    ~InfraredProcess() = default;

    virtual std::vector<char> DataProcessBase(const std::vector<char> &data) override
    {
        auto visiblelight = (VisibleLightInfrareMsg *)(data.data());

        TargDetectionImgMsg1 send_data;
        send_data.data_type = 1; // 红外
        // 图像名称不填
        send_data.image_source = 2;
        send_data.work_mode = 0;
        send_data.image_resolution = 0;

        uint32_t temp = swap_endian16(visiblelight->imageRows);
        send_data.image_rows = swap_endian32(temp);

        temp = swap_endian16(visiblelight->imageCols);
        send_data.image_cols = swap_endian32(temp);

        send_data.quantization_bits = visiblelight->quantization;
        send_data.total_package_num = visiblelight->totalPackageNum;
        memcpy(send_data.sub_package_index, visiblelight->subPackageSeq, 4);
        send_data.valid_data_length = visiblelight->dataLength;
        // 目标信息
        send_data.total_package_num = visiblelight->totalPackageNum;
        memcpy(send_data.sub_package_index, visiblelight->subPackageSeq, 4);
        send_data.valid_data_length = visiblelight->dataLength;

        std::vector<char> img_msg(data.data() + sizeof(VisibleLightInfrareMsg),
                                  data.data() + sizeof(VisibleLightInfrareMsg) +
                                      swap_endian32(visiblelight->dataLength));

        send_data.message_length = swap_endian32(sizeof(send_data) + img_msg.size() + sizeof(PkgTail));

        PkgTail tail;
        send_msg.resize(sizeof(send_data) + img_msg.size() + sizeof(PkgTail));
        memcpy(send_msg.data(), &send_data, sizeof(send_data));
        memcpy(send_msg.data() + sizeof(send_data), img_msg.data(), img_msg.size());
        memcpy(send_msg.data() + sizeof(send_data) + img_msg.size(), &tail, sizeof(PkgTail));

        return send_msg;
    };
};

#endif // INFRAEDPROCESS_HPP