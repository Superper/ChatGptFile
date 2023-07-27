#pragma once
#include "stdafx.h"
#include "opencv2/opencv.hpp"
#include <string>

#ifdef _DEBUG

	//以下为Dubug的lib 64位版本
	//#pragma comment(lib,"opencv_core2413d.lib")
	//#pragma comment(lib,"opencv_highgui2413d.lib")
	//#pragma comment(lib,"opencv_imgproc2413d.lib")

	//以下为Dubug的lib 32位版本
	#pragma comment(lib,"opencv_core248d.lib")
	#pragma comment(lib,"opencv_highgui248d.lib")
	#pragma comment(lib,"opencv_imgproc248d.lib")

#else

	//以下为Release的lib 64位版本
	//#pragma comment(lib,"opencv_core2413.lib")
	//#pragma comment(lib,"opencv_highgui2413.lib")
	//#pragma comment(lib,"opencv_imgproc2413.lib")

	//以下为Release的lib 32位版本
	#pragma comment(lib,"opencv_core248.lib")
	#pragma comment(lib,"opencv_highgui248.lib")
	#pragma comment(lib,"opencv_imgproc248.lib")
#endif

using namespace cv;  
using namespace std;   

struct tif_params
{
	UINT8 * dataMat;
	int dtHeight;
	int dtWidth;

};

struct SAR_VS_params
{
public:

	string input_file_name;
	string output_file_name;
	tif_params tifParams;
	int WinWidth;
	float looks;
	float gamma;
	float low;
	float high;
	int funtype;

	enum funname
	{
		AUTO,//自动对比度
		LEE,//Lee 滤波
		GAMMA,//Gamma 滤波
		KUAN, //Kuan滤波
		FROST,//Frost滤波
		SIGMA //Sigma滤波
	};
};
struct openCv_Proc
{
	enum funname
	{
		AUTO,//自动对比度
		LEE,//Lee 滤波
		GAMMA,//Gamma 滤波
		KUAN, //Kuan滤波
		FROST,//Frost滤波
		SIGMA //Sigma滤波
	};
	int WinWidth;
	float looks;
	float gamma;
	float low;
	float high;
	int funtype;

};

void SAR_VS_params_init(SAR_VS_params &params);

void Local_Funcs(SAR_VS_params param);
