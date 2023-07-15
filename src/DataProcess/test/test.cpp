#include "../SystemInterface.h"
#include <algorithm>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <vector>

int main()
{
    // VisibleLightInfrareMsg
    VisibleLightInfrareMsg visible_light;
    std::vector<uint8_t> visible_light_bytes(sizeof(visible_light));
    std::memcpy(visible_light_bytes.data(), reinterpret_cast<uint8_t *>(&visible_light), sizeof(visible_light));
    std::cout << "VisibleLightInfrareMsg:" << std::endl;
    for (auto byte : visible_light_bytes)
    {
        std::cout << std::hex << (int)byte << " ";
    }
    std::cout << std::endl << std::endl;

    // OthetLightMsg
    OthetLightMsg other_light;
    std::vector<uint8_t> other_light_bytes(sizeof(other_light));
    std::memcpy(other_light_bytes.data(), reinterpret_cast<uint8_t *>(&other_light), sizeof(other_light));
    std::cout << "OthetLightMsg:" << std::endl;
    for (auto byte : other_light_bytes)
    {
        std::cout << std::hex << (int)byte << " ";
    }
    std::cout << std::endl << std::endl;

    // PositionInformationOfImageFeaturePoints
    PositionInformationOfImageFeaturePoints position_info;
    std::vector<uint8_t> position_info_bytes(sizeof(position_info));
    std::memcpy(position_info_bytes.data(), reinterpret_cast<uint8_t *>(&position_info), sizeof(position_info));
    std::cout << "PositionInformationOfImageFeaturePoints:" << std::endl;
    for (auto byte : position_info_bytes)
    {
        std::cout << std::hex << (int)byte << " ";
    }
    std::cout << std::endl << std::endl;

    // Y9SARMsg1
    Y9SARMsg1 y9sar_msg_1;
    std::vector<uint8_t> y9sar_msg_1_bytes(sizeof(y9sar_msg_1));
    std::memcpy(y9sar_msg_1_bytes.data(), reinterpret_cast<uint8_t *>(&y9sar_msg_1), sizeof(y9sar_msg_1));
    std::cout << "Y9SARMsg1:" << std::endl;
    for (auto byte : y9sar_msg_1_bytes)
    {
        std::cout << std::hex << (int)byte << " ";
    }
    std::cout << std::endl << std::endl;

    // Y9SARMsg2
    Y9SARMsg2 y9sar_msg_2;
    std::vector<uint8_t> y9sar_msg_2_bytes(sizeof(y9sar_msg_2));
    std::memcpy(y9sar_msg_2_bytes.data(), reinterpret_cast<uint8_t *>(&y9sar_msg_2), sizeof(y9sar_msg_2));
    std::cout << "Y9SARMsg2:" << std::endl;
    for (auto byte : y9sar_msg_2_bytes)
    {
        std::cout << std::hex << (int)byte << " ";
    }
    std::cout << std::endl << std::endl;

    // OtherSARMsg1
    OtherSARMsg1 other_sar_msg_1;
    std::vector<uint8_t> other_sar_msg_1_bytes(sizeof(other_sar_msg_1));
    std::memcpy(other_sar_msg_1_bytes.data(), reinterpret_cast<uint8_t *>(&other_sar_msg_1), sizeof(other_sar_msg_1));
    std::cout << "OtherSARMsg1:" << std::endl;
    for (auto byte : other_sar_msg_1_bytes)
    {
        std::cout << std::hex << (int)byte << " ";
    }
    std::cout << std::endl << std::endl;

    // OtherSARMsg2
    OtherSARMsg2 other_sar_msg_2;
    std::vector<uint8_t> other_sar_msg_2_bytes(sizeof(other_sar_msg_2));
    std::memcpy(other_sar_msg_2_bytes.data(), reinterpret_cast<uint8_t *>(&other_sar_msg_2), sizeof(other_sar_msg_2));
    std::cout << "OtherSARMsg2:" << std::endl;
    for (auto byte : other_sar_msg_2_bytes)
    {
        std::cout << std::hex << (int)byte << " ";
    }
    std::cout << std::endl << std::endl;

    // TargDetectionImgMsg1
    TargDetectionImgMsg1 targ_det_img_msg_1;
    std::vector<uint8_t> targ_det_img_msg_1_bytes(sizeof(targ_det_img_msg_1));
    std::memcpy(targ_det_img_msg_1_bytes.data(), reinterpret_cast<uint8_t *>(&targ_det_img_msg_1),
                sizeof(targ_det_img_msg_1));
    std::cout << "TargDetectionImgMsg1:" << std::endl;
    for (auto byte : targ_det_img_msg_1_bytes)
    {
        std::cout << std::hex << (int)byte << " ";
    }
    std::cout << std::endl << std::endl;

    // TargDetectionImgMsg2
    TargDetectionImgMsg2 targ_det_img_msg_2;
    std::vector<uint8_t> targ_det_img_msg_2_bytes(sizeof(targ_det_img_msg_2));
    std::memcpy(targ_det_img_msg_2_bytes.data(), reinterpret_cast<uint8_t *>(&targ_det_img_msg_2),
                sizeof(targ_det_img_msg_2));
    std::cout << "TargDetectionImgMsg2:" << std::endl;
    for (auto byte : targ_det_img_msg_2_bytes)
    {
        std::cout << std::hex << (int)byte << " ";
    }
    std::cout << std::endl;

    std::condition_variable cv_;
    std::mutex mtx_;
    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock);
    return 0;
}
