#ifndef OTHERSARPROCESS_HPP
#define OTHERSARPROCESS_HPP
#include "ImgData.h"

class OtherSARProcess : public ImgData
{
    public:
    virtual std::vector<char> DataProcessBase(const std::vector<char> &data) override{

        send_msg.resize(data.size() - sizeof(PkgHeader)-20);
        memcpy(&send_msg, data.data() + sizeof(PkgHeader) + 20, data.size() - sizeof(PkgHeader) - 20);

        return send_msg;
    }
};

#endif // OTHERSARPROCESS_HPP