#ifndef IMGDATA_H
#define IMGDATA_H
#include "FTPUploader.h"
#include "SystemInterface.h"
#include <functional>
#include <map>
#include <vector>
#include <fstream>
#include <iostream>

using msg_callback = std::function<void(const std::string &msg)>;

class ImgData
{
  public:
    virtual ~ImgData(){};
    virtual std::vector<uint8_t> DataProcessBase(const std::vector<uint8_t> &data) = 0;

    void SetMsgCB(msg_callback mg)
    {
        mg_ = mg;
    }

  protected:
    FTPUploader ftp_;
    msg_callback mg_;
    std::vector<uint8_t> send_msg;
    std::map<uint16_t, std::vector<uint8_t>> msg_pkg_;
    std::string pkg_ftp_path_;
    void FtpImg(bool is_done, std::string file_path, uint16_t no, std::vector<uint8_t> file_data);

    PkgHeader header;
    static bool if_send_;
    static uint sar_c_;
    static uint inf_c_;
    static uint light_c_;
};

#endif // IMGDATA_H
