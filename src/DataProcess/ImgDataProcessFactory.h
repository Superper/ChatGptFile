#ifndef IMGDATAPROCESSFACTORY_H
#define IMGDATAPROCESSFACTORY_H

#include"OtherSARProcess.hpp"
#include"OtherLightProcess.hpp"
#include"VisibleLighetProcess.hpp"
#include"Y9SARProcess.hpp"
#include"InfraredProcess.hpp"
#include"Control.h"
#include<unordered_map>

class ImgDataProcessFactory
{
public:
    virtual ImgData* CreateImgDataProcess() = 0;
    virtual ~ImgDataProcessFactory() = default;
};

class OtherSARProcessFactory : public ImgDataProcessFactory
{
    public:
    virtual ImgData *CreateImgDataProcess() override
    {
        return new OtherSARProcess;
    }
};

class OtherLightProcessFactory : public ImgDataProcessFactory
{
    public:
    virtual ImgData *CreateImgDataProcess() override
    {
        return new OtherLightProcess;
    }
};

class VisibleLighetProcessFactory : public ImgDataProcessFactory
{
    public:
    virtual ImgData *CreateImgDataProcess() override
    {
        return new VisibleLighetProcess;
    }
};

class Y9SARProcessFactory : public ImgDataProcessFactory
{
    public:
    virtual ImgData *CreateImgDataProcess() override
    {
        return new Y9SARProcess;
    }
};

class InfraredProcessFactory : public ImgDataProcessFactory
{
    public:
    virtual ImgData *CreateImgDataProcess() override
    {
        return new InfraredProcess;
    }
};

class ControlFactory : public ImgDataProcessFactory
{
  public:
    virtual ImgData *CreateImgDataProcess() override
    {
        return new Control;
    }
};

#endif // IMGDATAPROCESSFACTORY_H
