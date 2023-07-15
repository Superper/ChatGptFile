#ifndef INFRAEDPROCESS_HPP
#define INFRAEDPROCESS_HPP
#include "ImgData.h"

class InfraredProcess : public ImgData
{
  public:
    InfraredProcess() = default;
    ~InfraredProcess() = default;



    virtual std::vector<uint8_t> DataProcessBase(const std::vector<uint8_t> &data) override
    {
        auto visiblelight = (VisibleLightInfrareMsg *)(data.data());

        // std::cout << "head.header: " << visiblelight->head.header << std::endl;
        // std::cout << "head.length: " << visiblelight->head.length << std::endl;
        // std::cout << "head.srcId: " << visiblelight->head.srcId << std::endl;
        // std::cout << "head.dstId: " << visiblelight->head.dstId << std::endl;
        // std::cout << "head.type: " << static_cast<int>(visiblelight->head.type) << std::endl;
        // std::cout << "head.msgType: " << static_cast<int>(visiblelight->head.msgType) << std::endl;
        // std::cout << "head.id: " << visiblelight->head.id << std::endl;
        // std::cout << "head.sendTime: " << visiblelight->head.sendTime << std::endl;
        // std::cout << "head.sendSeq: " << visiblelight->head.sendSeq << std::endl;

        // std::cout << "img_name: " << visiblelight->img_name << std::endl;
        // std::cout << "totalPackageNum: " << visiblelight->totalPackageNum << std::endl;
        // std::cout << "subPackageSeq: " << visiblelight->subPackageSeq << std::endl;
        // std::cout << "imageRows: " << visiblelight->imageRows << std::endl;
        // std::cout << "imageCols: " << visiblelight->imageCols << std::endl;
        // std::cout << "quantization: " << static_cast<int>(visiblelight->quantization) << std::endl;
        // std::cout << "reserved2: " << static_cast<int>(visiblelight->reserved2) << std::endl;
        // std::cout << "dataLength: " << visiblelight->dataLength << std::endl;

        TargDetectionImgMsg1 send_data;

        // 业务处理 文件存储
        std::string img_name(reinterpret_cast<char *>(visiblelight->img_name));
        std::vector<uint8_t> img_data(data.data() + sizeof(VisibleLightInfrareMsg),
                                      data.data() + sizeof(VisibleLightInfrareMsg) +
                                          swap_endian32(visiblelight->dataLength));
        auto sub_no = swap_endian32(visiblelight->subPackageSeq);
        auto total_no = swap_endian16(visiblelight->totalPackageNum);
        FtpImg(sub_no == total_no, img_name, sub_no, img_data);

        // 格式转化
        send_data.data_type = 1; // 红外
        send_data.image_source = 2;
        send_data.work_mode = 0;
        send_data.image_resolution = 0;

        uint32_t temp = swap_endian16(visiblelight->imageRows);
        send_data.image_rows = swap_endian32(temp);

        temp = swap_endian16(visiblelight->imageCols);
        send_data.image_cols = swap_endian32(temp);

        send_data.quantization_bits = visiblelight->quantization;
        send_data.total_package_num = visiblelight->totalPackageNum;
        send_data.valid_data_length = visiblelight->dataLength;
        send_data.total_package_num = visiblelight->totalPackageNum;

        if (swap_endian32(visiblelight->dataLength) > 256)
        {
            send_data.valid_data_length = swap_endian32(256);
        }
        else
        {
            send_data.valid_data_length = visiblelight->dataLength;
        }

        std::vector<uint8_t> img_msg(data.data() + sizeof(VisibleLightInfrareMsg),
                                     data.data() + sizeof(VisibleLightInfrareMsg) +
                                         swap_endian32(visiblelight->dataLength));

        send_data.message_length = swap_endian32(sizeof(send_data) + img_msg.size() + sizeof(PkgTail));

        PkgTail tail;
        send_msg.resize(sizeof(send_data) + img_msg.size() + sizeof(PkgTail));
        memcpy(send_msg.data(), &send_data, sizeof(send_data));
        memcpy(send_msg.data() + sizeof(send_data), img_msg.data(), img_msg.size());
        memcpy(send_msg.data() + sizeof(send_data) + img_msg.size(), &tail, sizeof(PkgTail));

        if (if_send_)
        {
            mg_("inf-" + std::to_string(++inf_c_));
            return send_msg;
        }
        else
        {
            std::vector<uint8_t> temp;
            return temp;
        }
    };
};

#endif // INFRAEDPROCESS_HPP