#pragma once
#include "stdafx.h"
#include "opencv2/opencv.hpp"
#include <string>

#ifdef _DEBUG

	//����ΪDubug��lib 64λ�汾
	//#pragma comment(lib,"opencv_core2413d.lib")
	//#pragma comment(lib,"opencv_highgui2413d.lib")
	//#pragma comment(lib,"opencv_imgproc2413d.lib")

	//����ΪDubug��lib 32λ�汾
	#pragma comment(lib,"opencv_core248d.lib")
	#pragma comment(lib,"opencv_highgui248d.lib")
	#pragma comment(lib,"opencv_imgproc248d.lib")

#else

	//����ΪRelease��lib 64λ�汾
	//#pragma comment(lib,"opencv_core2413.lib")
	//#pragma comment(lib,"opencv_highgui2413.lib")
	//#pragma comment(lib,"opencv_imgproc2413.lib")

	//����ΪRelease��lib 32λ�汾
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
		AUTO,//�Զ��Աȶ�
		LEE,//Lee �˲�
		GAMMA,//Gamma �˲�
		KUAN, //Kuan�˲�
		FROST,//Frost�˲�
		SIGMA //Sigma�˲�
	};
};
struct openCv_Proc
{
	enum funname
	{
		AUTO,//�Զ��Աȶ�
		LEE,//Lee �˲�
		GAMMA,//Gamma �˲�
		KUAN, //Kuan�˲�
		FROST,//Frost�˲�
		SIGMA //Sigma�˲�
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
