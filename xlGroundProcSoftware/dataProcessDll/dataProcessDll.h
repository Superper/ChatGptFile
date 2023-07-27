// ************************************************
//					Author: Robles 
//					Date  : 2016/6/25
//   Electronic Institude of Chinese Sceiene Academy 
//   Update: 2016/7/12 added geotiff generation and release for Win32 platform
//   Update: 2016/7/14 checked SAR-IMAGE mode and SAR-GMTI mode 
//   Update: 2016/7/20 checked SAR-GMTI mode and WAS-MTI mode
//   Update: 2016/10/27 check SAR-GMTI real time mode 
//   Update: 2016/11/30 add image merge modual for image preprocessing
//   Update: 2020/03/04 adjust for FPGA V4 code 
//   Update: 2022/08/19 added creatTiff for bigger data block 
// **************************************************

#define UINT8  unsigned __int8

// &&&&& 离线图像处理进度反馈说明 &&&&&&&

// 离线图像模式处理的时候会在程序的当前目录下产生一个名为offline_prog.txt的文本
// 处理时将写入当前处理RAW文件名；
// 处理完成时将写入结束符号end_of_proc



// *******  WAS-GMT模式点迹离线处理入口函数 *********
extern "C" _declspec(dllexport) int _stdcall WasGMTIOfflineProc(const char *strSource, const char *strDest);
// strSource --- WAS-GMTI模式（只有点迹）点迹文件名；
// strDest   --- 输出目录


// ******* SAR-IMAGE mode and SAR-GMTI mode 离线处理函数入口函数 ********** 
extern "C" _declspec(dllexport) int _stdcall ImageModeOfflineProc(const char *strImageFolder, const char *strDest);
// strImageFolder --- 点迹和RAW图像所在的目录；
// strDest   --- 输出目录







// ******* SAR-IMAGE mode and SAR-GMTI mode 实时处理函数入口函数 **********
extern "C" _declspec(dllexport) int _stdcall ImageModeOnlineProc(UINT8 *data, __int64 dataLen, char *szDstPath, char *szTrackFile);
// data		   --- RAW图像内存地址；
// datalen	   --- RAW图像大小；
// szDstPath   --- 输出目录
// szTrackFile --- 实时点迹文件路径


extern "C" _declspec(dllexport) int _stdcall ImageModeOnlineProc_GAI(UINT8 *data, __int64 dataLen, UINT8 blocknum, char *szDstPath, char *szTrackFile);
// data		   --- 图像内存起始地址；
// datalen	   --- 单个RAW图像大小；
// blocknum    --- 每次内存RAW图像块数
// szDstPath   --- 输出目录
// szTrackFile --- 实时点迹文件路径


// ******* WAS-GMTI点迹实时处理主函数入口  ************
extern "C" _declspec(dllexport) int _stdcall WasMTIOnlineProc(UINT8 *dataAr, const char *strDest, __int64 len);
// dataAr  --- 点迹数据内存地址
// strDest --- 输出目录
// len     --- 点迹数据长度



// ************  预处理软件调用函数  *********************

typedef struct
{
	//bool blImgOrigin,blImgMerge,blTxt,blDat;
	__int32 blImgOrigin;
	__int32 blImgMerge;
	__int32 blTxt;
	__int32 blDat;
	__int32 blSingle;
	__int32 blMultiple;
	__int32 blHjtxt;

}stGMTIProcContent;

struct imageProcParsST {

public:
	//是否去斑 辐射校正 几何校正 直方图均衡
	__int32 blRemoveSpots;
	__int32 blAmpCor;
	__int32 blGeoCor;
	__int32 blHs;

	double heading_angle;
	double ra_res;
	double r_near;
	double href;
	double r_ang;
	double ra_beam_ang;
};

// ******* 预处理软件点迹DAT文件生成TXT函数 check by luo 2016/06/07  *********
extern "C" _declspec(dllexport) void _stdcall gmti6SuoDatRestore(char *strSource, char *strDest, stGMTIProcContent prcCont);

// ******* 预处理软件图像灰度和校正操作函数 check by luo 2016/06/07 ****************
extern "C" _declspec(dllexport) void _stdcall imagePreProcOut(char *strImage, char *strDest, imageProcParsST procPars);

// ******* 图像拼接 check by luo 2016/11/28 ****************
// extern "C" _declspec(dllexport) void _stdcall imageMerge(string *strSource, char *strDest, int N);