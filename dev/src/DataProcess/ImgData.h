#ifndef IMGDATA_H
#define IMGDATA_H
#include"SystemInterface.h"

class ImgData
{
  public:
    virtual ~ImgData(){};
    virtual std::vector<char> DataProcessBase(const std::vector<char> &data) = 0;
  protected:
    std::vector<char> send_msg;
};

#endif  // IMGDATA_H