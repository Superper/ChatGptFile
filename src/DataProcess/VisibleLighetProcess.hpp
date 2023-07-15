#ifndef VisibleLighetProcess_HPP
#define VisibleLighetProcess_HPP
#include "ImgData.h"
class VisibleLighetProcess : public ImgData
{

  public:
    VisibleLighetProcess() = default;
    ~VisibleLighetProcess() = default;

    virtual std::vector<uint8_t> DataProcessBase(const std::vector<uint8_t> &data) override
    {
        auto visiblelight = (VisibleLightInfrareMsg *)data.data();

        // ftp
        std::string img_name(reinterpret_cast<char *>(visiblelight->img_name));
        auto dl = swap_endian32(visiblelight->dataLength);
        std::vector<uint8_t> img_data(data.data() + sizeof(VisibleLightInfrareMsg),
                                      data.data() + sizeof(VisibleLightInfrareMsg) +dl);
        auto sub_no = swap_endian32(visiblelight->subPackageSeq);
        auto total_no = swap_endian16(visiblelight->totalPackageNum);
        FtpImg(sub_no == total_no -1 , img_name, sub_no, img_data);

        auto aa = 1;
        // 打印

        // // 输出PkgHeader字段的值
        // std::cout << "header: " << std::hex << visiblelight->head.header << std::endl;
        // std::cout << "length: " << visiblelight->head.length << std::endl;
        // std::cout << "srcId: " << std::hex << visiblelight->head.srcId << std::endl;
        // std::cout << "dstId: " << std::hex << visiblelight->head.dstId << std::endl;
        // std::cout << "type: " << std::hex << static_cast<int>(visiblelight->head.type) << std::endl;
        // std::cout << "msgType: " << std::hex << static_cast<int>(visiblelight->head.msgType) << std::endl;
        // std::cout << "id: " << std::hex << visiblelight->head.id << std::endl;
        // std::cout << "sendTime: " << visiblelight->head.sendTime << std::endl;
        // std::cout << "sendSeq: " << visiblelight->head.sendSeq << std::endl;

        // // 输出VisibleLightInfrareMsg字段的值
        // std::cout << "img_name: " << reinterpret_cast<char *>(visiblelight->img_name) << std::endl;
        // std::cout << "totalPackageNum: " << visiblelight->totalPackageNum << std::endl;
        // std::cout << "subPackageSeq: ";
        // std::cout << std::endl;
        // std::cout << "imageRows: " << visiblelight->imageRows << std::endl;
        // std::cout << "imageCols: " << visiblelight->imageCols << std::endl;
        // std::cout << "quantization: " << static_cast<int>(visiblelight->quantization) << std::endl;
        // std::cout << "reserved2: " << static_cast<int>(visiblelight->reserved2) << std::endl;
        // std::cout << "dataLength: " << visiblelight->dataLength << std::endl;

        // TargDetectionImgMsg1 send_data;
        // send_data.data_type = 2; // 光学
        // // 图像名称不填
        // send_data.image_source = 2;
        // send_data.work_mode = 0;
        // send_data.image_resolution = 0;

        // uint32_t temp = swap_endian16(visiblelight->imageRows);
        // send_data.image_rows = swap_endian32(temp);

        // temp = swap_endian16(visiblelight->imageCols);
        // send_data.image_cols = swap_endian32(temp);

        // send_data.quantization_bits = visiblelight->quantization;

        // // 目标信息
        // ftp_.uploadData(data.data() + 27 + 14, 37,
        //                 "ftp://102.182.187.152/file.bin", "crushadmin:1223");

        // send_data.total_package_num = visiblelight->totalPackageNum;
        // send_data.sub_package_index= visiblelight->subPackageSeq;

        // if (swap_endian32(visiblelight->dataLength) > 256)
        // {
        //     send_data.valid_data_length = swap_endian32(256);
        // }
        // else
        // {
        //     send_data.valid_data_length = visiblelight->dataLength;
        // }

        // std::vector<uint8_t> img_msg(data.data() + sizeof(VisibleLightInfrareMsg),
        //                           data.data() + sizeof(VisibleLightInfrareMsg) +
        //                               swap_endian32(send_data.valid_data_length));

        // send_data.message_length = swap_endian32(sizeof(send_data) + img_msg.size() + sizeof(PkgTail)); // todo
        // 有问题

        // PkgTail tail;
        // send_msg.resize(sizeof(send_data) + img_msg.size() + sizeof(PkgTail));
        // memcpy(send_msg.data(), &send_data, sizeof(send_data));
        // memcpy(send_msg.data() + sizeof(send_data), img_msg.data(), img_msg.size());
        // memcpy(send_msg.data() + sizeof(send_data) + img_msg.size(), &tail, sizeof(PkgTail));

        // if (if_send_)
        // {
        //     mg_("lig-" + std::to_string(++light_c_));
        //     return send_msg;
        // }
        // else
        // {
        //     std::vector<uint8_t> temp;
        //     return temp;
        // }
        std::vector<uint8_t> temp;
        return temp;
    };
};

#endif // VisibleLighetProcess_HPP