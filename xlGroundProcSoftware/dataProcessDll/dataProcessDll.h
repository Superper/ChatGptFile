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

// &&&&& ����ͼ������ȷ���˵�� &&&&&&&

// ����ͼ��ģʽ�����ʱ����ڳ���ĵ�ǰĿ¼�²���һ����Ϊoffline_prog.txt���ı�
// ����ʱ��д�뵱ǰ����RAW�ļ�����
// �������ʱ��д���������end_of_proc



// *******  WAS-GMTģʽ�㼣���ߴ�����ں��� *********
extern "C" _declspec(dllexport) int _stdcall WasGMTIOfflineProc(const char *strSource, const char *strDest);
// strSource --- WAS-GMTIģʽ��ֻ�е㼣���㼣�ļ�����
// strDest   --- ���Ŀ¼


// ******* SAR-IMAGE mode and SAR-GMTI mode ���ߴ�������ں��� ********** 
extern "C" _declspec(dllexport) int _stdcall ImageModeOfflineProc(const char *strImageFolder, const char *strDest);
// strImageFolder --- �㼣��RAWͼ�����ڵ�Ŀ¼��
// strDest   --- ���Ŀ¼







// ******* SAR-IMAGE mode and SAR-GMTI mode ʵʱ��������ں��� **********
extern "C" _declspec(dllexport) int _stdcall ImageModeOnlineProc(UINT8 *data, __int64 dataLen, char *szDstPath, char *szTrackFile);
// data		   --- RAWͼ���ڴ��ַ��
// datalen	   --- RAWͼ���С��
// szDstPath   --- ���Ŀ¼
// szTrackFile --- ʵʱ�㼣�ļ�·��


extern "C" _declspec(dllexport) int _stdcall ImageModeOnlineProc_GAI(UINT8 *data, __int64 dataLen, UINT8 blocknum, char *szDstPath, char *szTrackFile);
// data		   --- ͼ���ڴ���ʼ��ַ��
// datalen	   --- ����RAWͼ���С��
// blocknum    --- ÿ���ڴ�RAWͼ�����
// szDstPath   --- ���Ŀ¼
// szTrackFile --- ʵʱ�㼣�ļ�·��


// ******* WAS-GMTI�㼣ʵʱ�������������  ************
extern "C" _declspec(dllexport) int _stdcall WasMTIOnlineProc(UINT8 *dataAr, const char *strDest, __int64 len);
// dataAr  --- �㼣�����ڴ��ַ
// strDest --- ���Ŀ¼
// len     --- �㼣���ݳ���



// ************  Ԥ����������ú���  *********************

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
	//�Ƿ�ȥ�� ����У�� ����У�� ֱ��ͼ����
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

// ******* Ԥ��������㼣DAT�ļ�����TXT���� check by luo 2016/06/07  *********
extern "C" _declspec(dllexport) void _stdcall gmti6SuoDatRestore(char *strSource, char *strDest, stGMTIProcContent prcCont);

// ******* Ԥ�������ͼ��ҶȺ�У���������� check by luo 2016/06/07 ****************
extern "C" _declspec(dllexport) void _stdcall imagePreProcOut(char *strImage, char *strDest, imageProcParsST procPars);

// ******* ͼ��ƴ�� check by luo 2016/11/28 ****************
// extern "C" _declspec(dllexport) void _stdcall imageMerge(string *strSource, char *strDest, int N);