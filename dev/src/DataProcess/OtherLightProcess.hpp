#ifndef OTHERLIGHTPROCESS_HPP
#define OTHERLIGHTPROCESS_HPP
#include "ImgData.h"

class OtherLightProcess : public ImgData
{
    public:
    OtherLightProcess() = default;
    ~OtherLightProcess() = default;

    virtual std::vector<char> DataProcessBase(const std::vector<char> &data) override
    {
        auto otherlight = (OthetLightMsg *)data.data();

        TargDetectionImgMsg1 send_data;
        send_data.data_type = 2; // 光学

        memcpy(send_data.image_name, otherlight->image_name, 256);

        switch (otherlight->image_source)
        {
        case 1: // 乌镇7
            send_data.image_source = 3;
            break;

        case 2: // 卫星
            send_data.image_source = 1;
            break;
        case 3: // 其他无人机
            send_data.image_source = 3;
            break;
        default:
            send_data.image_source = 0;
            break;
        }

        send_data.work_mode = 0;
        send_data.image_resolution = otherlight->image_resolution;
        send_data.image_rows = otherlight->image_row_count;
        send_data.image_cols = otherlight->image_column_count;
        send_data.quantization_bits = otherlight->quantization_bits;
        send_data.total_package_num = otherlight->total_packets;
        // todo::子包序号问题
        send_data.valid_data_length = otherlight->data_length;

        std::vector<char> img_msg(data.data() + sizeof(OthetLightMsg),
                                  data.data() + sizeof(OthetLightMsg) + swap_endian32(otherlight->data_length));

        send_data.message_length = swap_endian32(sizeof(send_data) + img_msg.size() + sizeof(PkgTail));

        PkgTail tail;
        send_msg.resize(sizeof(send_data) + img_msg.size() + sizeof(PkgTail));
        memcpy(send_msg.data(), &send_data, sizeof(send_data));
        memcpy(send_msg.data() + sizeof(send_data), img_msg.data(), img_msg.size());
        memcpy(send_msg.data() + sizeof(send_data) + img_msg.size(), &tail, sizeof(PkgTail));

        return send_msg;
    };
};

#endif