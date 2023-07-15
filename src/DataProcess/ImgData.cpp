#include "ImgData.h"

bool ImgData::if_send_ = false;
uint ImgData::sar_c_ = 0;
uint ImgData::inf_c_ = 0;
uint ImgData::light_c_ = 0;

void ImgData::FtpImg(bool is_done, std::string file_path, uint16_t no, std::vector<uint8_t> file_data)
{
    file_path = "/Users/zhx/Work/ZN/dev/img/"+file_path;
    if (is_done)
    {
        msg_pkg_[no] = file_data;
        std::vector<uint8_t> ftp_data;
        for (auto data : msg_pkg_)
        {
            ftp_data.insert(ftp_data.end(), data.second.begin(), data.second.end());
        }

        // ftp_.uploadData(ftp_data.data(), ftp_data.size(), "ftp://ftp.example.com/upload/file.bin",
        // "username:password");

        std::ofstream outfile;
        outfile.open(pkg_ftp_path_);
        if (outfile.is_open())
        {
            for (const auto &num : ftp_data)
            {
                outfile << num;
            }
            outfile.close();
        }
        else
        {
            std::cout << "Unable to open file for writing." << std::endl;
        }

        msg_pkg_.clear();
    }
    else
    {
        // 非正常结束，前包没有发完情况，直接存储文件
        if (no == 0 && file_path != pkg_ftp_path_ && !msg_pkg_.empty())
        {
            std::vector<uint8_t> ftp_data;
            for (auto data : msg_pkg_)
            {
                ftp_data.insert(ftp_data.end(), data.second.begin(), data.second.end());
            }
            // ftp_.uploadData(ftp_data.data(), ftp_data.size(), "ftp://ftp.example.com/upload/file.bin",
            // "username:password");
            std::ofstream outfile;
            outfile.open(pkg_ftp_path_);
            if (outfile.is_open())
            {
                for (const auto &num : ftp_data)
                {
                    outfile << num;
                }
                outfile.close();
            }
            else
            {
                std::cout << "Unable to open file for writing." << std::endl;
            }

            msg_pkg_.clear();
            msg_pkg_[no] = file_data;
        }
        pkg_ftp_path_ = file_path;
        msg_pkg_[no] = file_data;
    }
}
