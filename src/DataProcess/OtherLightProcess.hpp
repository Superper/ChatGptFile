#ifndef OTHERLIGHTPROCESS_HPP
#define OTHERLIGHTPROCESS_HPP
#include "ImgData.h"

class OtherLightProcess : public ImgData
{
  public:
    OtherLightProcess() = default;
    ~OtherLightProcess() = default;

    virtual std::vector<uint8_t> DataProcessBase(const std::vector<uint8_t> &data) override
    {
        auto otherlight = (OthetLightMsg *)data.data();

        // 输出PkgHeader字段的值
        std::cout << "header: " << std::hex << otherlight->head.header << std::endl;
        std::cout << "length: " << otherlight->head.length << std::endl;
        std::cout << "srcId: " << std::hex << otherlight->head.srcId << std::endl;
        std::cout << "dstId: " << std::hex << otherlight->head.dstId << std::endl;
        std::cout << "type: " << std::hex << static_cast<int>(otherlight->head.type) << std::endl;
        std::cout << "msgType: " << std::hex << static_cast<int>(otherlight->head.msgType) << std::endl;
        std::cout << "id: " << std::hex << otherlight->head.id << std::endl;
        std::cout << "sendTime: " << otherlight->head.sendTime << std::endl;
        std::cout << "sendSeq: " << otherlight->head.sendSeq << std::endl;

        // 输出OthetLightMsg字段的值
        std::cout << "image_name: " << reinterpret_cast<char *>(otherlight->image_name) << std::endl;
        std::cout << "image_type: " << static_cast<int>(otherlight->image_type) << std::endl;
        std::cout << "image_source: " << static_cast<int>(otherlight->image_source) << std::endl;
        std::cout << "image_resolution: " << otherlight->image_resolution << std::endl;
        std::cout << "image_row_count: " << otherlight->image_row_count << std::endl;
        std::cout << "image_column_count: " << otherlight->image_column_count << std::endl;
        std::cout << "quantization_bits: " << static_cast<int>(otherlight->quantization_bits) << std::endl;
        std::cout << "total_packets: " << otherlight->total_packets << std::endl;
        std::cout << "packet_index: " << otherlight->packet_index << std::endl;
        std::cout << "reserved: " << static_cast<int>(otherlight->reserved) << std::endl;
        std::cout << "data_length: " << otherlight->data_length << std::endl;




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

        if (swap_endian32(otherlight->data_length) > 256)
        {
            send_data.valid_data_length = swap_endian32(256);
        }
        else
        {
            send_data.valid_data_length = otherlight->data_length;
        }

        std::vector<uint8_t> img_msg(data.data() + sizeof(OthetLightMsg),
                                  data.data() + sizeof(OthetLightMsg) + swap_endian32(otherlight->data_length));

        send_data.message_length = swap_endian32(sizeof(send_data) + img_msg.size() + sizeof(PkgTail));

        PkgTail tail;
        send_msg.resize(sizeof(send_data) + img_msg.size() + sizeof(PkgTail));
        memcpy(send_msg.data(), &send_data, sizeof(send_data));
        memcpy(send_msg.data() + sizeof(send_data), img_msg.data(), img_msg.size());
        memcpy(send_msg.data() + sizeof(send_data) + img_msg.size(), &tail, sizeof(PkgTail));

        if (if_send_)
        {
            mg_("lig-" + std::to_string(++light_c_));
            return send_msg;
        }
        else
        {
            std::vector<uint8_t> temp;
            return temp;
        }
    };
};

#endif