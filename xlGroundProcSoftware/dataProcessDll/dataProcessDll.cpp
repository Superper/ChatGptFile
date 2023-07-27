// dataProcessDll.cpp : 定义 DLL 应用程序的导出函数。
//Updated by Yunhua-Luo 2016/2/26 V15 stable version Released
// 2016/03/02 added real-time SAR-GMTI restore  
// updated at 2016/3/17 simplfy the code  
// updated at 2016/7/21 
// updated by Luo 2018/11/9 for image location improve
// updated by Luo 2019/05/04 

#include "stdafx.h"
#include "parsExtraction.h"

char DllPath[1024];

// functions declaration 

int GetTrackDataFromFile(char* trackFile, BYTE* trackBuf);

int PickupSmallPic(BYTE *imageBuffer, int bufSize, long aimPRF);

// *** 处理进度的文本参数传递 ************
void WriteProgInfo(const char *str);

// *** SAR-IMAGE 离线图像处理函数 ********
int creatTIFF(UINT8 *dataAr, const char *strDest, __int64 len);

// *** SAR-IMAGE 实时图像处理函数 ********
int creatTIFF_Online(UINT8 *dataAr, const char *strDest, __int64 len, UINT8 blocknum);

// *** SAR-GMTI 实时图像处理函数V2.0 ********
int sarGMTIInfoRealMergeGai(UINT8 *data, __int64 dataLen, char *szDstPath, char *szTrackFile);

// *** WAS-GMTI 处理功能函数接口 ******
int WasGMTIDatProc(UINT8 *dataAr, const char *strDest, int len);

// *** SAR-GMTI离线处理子函数 ********
void sarGMTIOfflineProc(const char *strImageFolder, const char *strAim, const char *strDest);


// ++++++++++++++++ 处理子函数 +++++++++++++++++++++++++++++++++++++++++++++++++++

// Get lastest new track % Write by Dr.Luo 2016/11/2
int GetTrackDataFromFile(char* trackFile, BYTE* trackBuf)
{ // CHECK OK

	int TRACK_DATA_SIZE = BYTE_PER_TAR_MTI * 8192;
	TRACK_DATA_SIZE = BYTE_PER_TAR_MTI * 3072;
	__int64 TRACK_SIZE = TRACK_AUX_SIZE + TRACK_DATA_SIZE;

	commonFuncs cmFuncs;

	cmFuncs.logRecords("Entry read track data ...", 0);

	static __int64 fileSize_prev = 0;
	__int64 fileSize;

	FILE *fp;
	if ((fp = fopen(trackFile, "rb")) == NULL)
	{
		cmFuncs.logRecords("Open track file failed ...", 0);
		return -1;
	}
	_fseeki64(fp, 0, SEEK_END);
	fileSize = _ftelli64(fp);
	fseek(fp, 0, SEEK_SET);

	if (fileSize == fileSize_prev || fileSize == 0) // has not updated 
	{
		fclose(fp);
		return 0;
	}

	// cmFuncs.logRecords("Entry read track data step 2 ...", 0);

	fileSize_prev = fileSize;

	BYTE* fileBuffer = new BYTE[fileSize];
	if (fileBuffer == NULL)
	{
		cmFuncs.logRecords("Track file buffer allocate failed ...", 0);
		fclose(fp);
		return 0;
	}
	fread(fileBuffer, 1, fileSize, fp);
	fclose(fp);

	// cmFuncs.logRecords("Entry read track data step 3 ...", 0);

	// Get lastest N Tracks 
	memset(trackBuf, 0, sizeof(BYTE)*__int64(TRACK_BUFFER_SIZE)*TRACK_SIZE);
	BYTE* pt = fileBuffer;

	__int64 TrackBufCnt = 0;
	for (__int64 n = fileSize - 4; n >= 0; n--)
	{
		if (pt[n] == 0x01 && pt[n + 1] == 0xDC && pt[n + 2] == 0xEF)
		{
			if (((cmFuncs.getSarModel(pt + n)) == 3) && (n + TRACK_SIZE) <= fileSize)
			{
				memcpy(trackBuf + TRACK_SIZE*TrackBufCnt, pt + n, sizeof(BYTE)*TRACK_SIZE);
				TrackBufCnt++;
				if (TrackBufCnt == TRACK_BUFFER_SIZE) break;
			}
		}

	}

	delete[] fileBuffer; fileBuffer = NULL;
	cmFuncs.logRecords(" UpdateTrackData sucessed ... ", TrackBufCnt);

	return TrackBufCnt;

}

int PickupSmallPic(BYTE *imageBuffer, int bufSize, long aimPRF)
{
	// STEP 2048
	commonFuncs cmFuncs;
	for (int n = 0; n < bufSize; n++)
	{
		BYTE *pBuf = imageBuffer + n*MAX_IMAGE_LEN;
		if (*pBuf == 0) continue;
		long imgPRF = cmFuncs.getPRFNum(pBuf);
		if (imgPRF == aimPRF)return n;
	}
	return -1;
}

// ****** SAR/GMTI模式离线处理子函数 ***********	
void WriteProgInfo(const char *str)
{
	ofstream f1(DllPath, ios::app);
	if (!f1) return;
	f1 << str << endl;
	f1.close();
}

// ****** SAR/GMTI模式离线处理子函数 ***********
void sarGMTIOfflineProc(const char *strImageFolder, const char *strAim, const char *strDest)
{
	commonFuncs cmFuncs;
	static int loopNum = 0;

	int hdLen = 512;//帧头有512个字节
	int hdAimLen = 1024;//帧头有1024个字节
	bool match_flag = false;

	queue<std::string> fls;
	WIN32_FIND_DATA FindData;
	HANDLE hError;

	/* ======== add by zhang @ 20160224 ======== */

	/* STEP.1 create folder 'SAR-GMTI' */
	char *outputPath = new char[512];
	ZeroMemory(outputPath, 512);
	int len = strlen(strDest);
	CopyMemory(outputPath, strDest, len);
	if (outputPath[len - 1] == '\\')
	{
		len += sprintf(outputPath + len, "SAR-GMTI\\");

	}
	else
	{
		len += sprintf(outputPath + len, "\\SAR-GMTI\\");
	}
	_mkdir((const char *)outputPath);

	/* create sub folder 'SAR-GMTI\TUXIANG' */
	char imagePath[512];
	CopyMemory(imagePath, outputPath, 512);
	sprintf(imagePath + len, "TUXIANG\\");
	_mkdir((const char *)imagePath);

	/* create sub folder 'SAR-GMTI\DIANJI' */
	char infoPath[512];
	CopyMemory(infoPath, outputPath, 512);
	sprintf(infoPath + len, "DIANJI\\");
	_mkdir((const char *)infoPath);

	/* create sub folder 'SAR-GMTI\DIEJIATU' */
	char overlayPath[512];
	CopyMemory(overlayPath, outputPath, 512);
	sprintf(overlayPath + len, "DIEJIATU\\");
	_mkdir((const char *)overlayPath);

	delete[] outputPath;
	outputPath = NULL;

	/* STEP.2 Search for raw-images */
	char FilePathName[1024];
	// 构造路径
	char FullPathName[1024];
	strcpy(FilePathName, strImageFolder);
	strcat(FilePathName, "\\*.raw");  // modified by luo yunhua 

	hError = ::FindFirstFile(FilePathName, &FindData);
	if (hError == INVALID_HANDLE_VALUE)
	{
		cmFuncs.logRecords("can not find raw image file in folder ... ", 0.0);
		return;
	}

	while (1)
	{
		string str(FindData.cFileName);

		if (str.size()>4 && (str.substr(str.size() - 4, 4) == ".raw"))
		{
			fls.push(str);
			//fls.push_back(str);
		}

		if (!(::FindNextFile(hError, &FindData))) break;
	}

	if (fls.empty()) return;

	/* STEP.3 Start to match raw-images */

	string flFolder(strImageFolder);
	string ss = fls.front();
	fls.pop();
	string imageFl = flFolder + "\\" + ss;

	WriteProgInfo(ss.c_str()); // write to progress 

							   //图像文件
	FILE  *FLImage = fopen(imageFl.c_str(), "rb");
	UINT8 *headerImageAr = new UINT8[hdLen];
	UINT8 *lastImageAr = new UINT8[hdLen];
	UINT8 *arImageFrame;

	//图像数组大小
	UINT32 imageArSize;
	int endMarkImage = feof(FLImage);

	endMarkImage = cmFuncs.searchHeadPosition(FLImage);
	if (endMarkImage != 0)
	{
		delete[]headerImageAr;
		headerImageAr = NULL;
		fclose(FLImage);

		delete[] lastImageAr;
		lastImageAr = NULL;

		cmFuncs.logRecords("can not find guidecode for image file ... ", endMarkImage);
		return;
	}

	//读取图像信息
	//第一次
	fread(headerImageAr, sizeof(UINT8), hdLen, FLImage);
	UINT32 prfCntImage = cmFuncs.getPRFNum(headerImageAr);
	int mView = cmFuncs.getMViewNum(headerImageAr);

	int picsNum = 64 / mView; // Num to merge 
	int rgPoints = cmFuncs.getRangePoints(headerImageAr);
	UINT32  lenImageFrame = rgPoints*hdLen;
	imageArSize = lenImageFrame*picsNum;

	arImageFrame = new UINT8[imageArSize];
	memset(arImageFrame, sizeof(UINT8), imageArSize);
	int fdImageRes = fread(arImageFrame, sizeof(UINT8), lenImageFrame, FLImage);
	//int arImgActualLen=1;
	fclose(FLImage);

	if (fdImageRes<lenImageFrame)
	{
		delete[]headerImageAr;
		headerImageAr = NULL;

		delete[] lastImageAr;
		lastImageAr = NULL;

		delete[]arImageFrame;
		arImageFrame = NULL;

		return;
	}

	//目标文件	
	FILE *FLAim = fopen(strAim, "rb");
	int endAimMark = feof(FLAim);
	endAimMark = cmFuncs.searchHeadPosition(FLAim);

	//没有搜索到，但已经到文件尾
	if (endAimMark != 0)
	{
		delete[]headerImageAr;
		headerImageAr = NULL;

		delete[] lastImageAr;
		lastImageAr = NULL;

		delete[]arImageFrame;
		arImageFrame = NULL;
		fclose(FLAim);
		return;
	}

	//读取目标信息
	UINT32  lenAimFrame;// add by zhang : TODO lenAimFrame will change size in future

	lenAimFrame = hdAimLen + BYTE_PER_TAR_MTI * 3072;
	UINT8 *arAimFrame = new UINT8[lenAimFrame];
	memset(arAimFrame, sizeof(UINT8), lenAimFrame);
	int fdAimRes = fread(arAimFrame, sizeof(UINT8), lenAimFrame, FLAim);

	if (fdAimRes<lenAimFrame)
	{
		delete[] headerImageAr;
		headerImageAr = NULL;

		delete[] arImageFrame;
		arImageFrame = NULL;

		delete[] lastImageAr;
		lastImageAr = NULL;

		delete[] arAimFrame;
		arAimFrame = NULL;

		return;
	}

	UINT32 prfCntAim = cmFuncs.getPRFNum(arAimFrame);

	imgProc procImg;
	SAR_VS_params stPars;

	/* ==== add by zhang : reset power on time ==== */
	static int lastPowerOnTime = -1;
	int currentPowerOnTime = cmFuncs.getPowerOnTime(arAimFrame);
	if (currentPowerOnTime != lastPowerOnTime)
	{
		lastPowerOnTime = currentPowerOnTime;
		loopNum = 0;
	}

	/* ============================================= */
	// Calculate multilook coef 
	float az_ml2_coef = 1.0;
	if (ACCURATE_ML)
	{
		sarImageInfo img_info0(headerImageAr);
		az_ml2_coef = img_info0.ml2_coef;
	}

	do
	{
		//cmFuncs.logRecords("******** loopNum ************: ",loopNum);

		while (prfCntAim>prfCntImage)  // read prfcnt from image file
		{
			cmFuncs.logRecords("Target prfcnt is bigger than image prfcnt, read prfcnt from image file ... ", 1);
			if (fls.empty())
			{
				//有可能最后一幅图像不能生成 在大循环后边,可根据数组实际长度进行判断
				break;
			}
			ss = fls.front();
			fls.pop();
			imageFl = flFolder + "\\" + ss;
			FLImage = fopen(imageFl.c_str(), "rb");
			fread(headerImageAr, sizeof(UINT8), hdLen, FLImage);
			prfCntImage = cmFuncs.getPRFNum(headerImageAr);
			fdImageRes = fread(arImageFrame, sizeof(UINT8), lenImageFrame, FLImage);
			fclose(FLImage);
		}

		if (prfCntAim<prfCntImage) // Read prfcnt from aimfile;
		{
			cmFuncs.logRecords("Target prfcnt is smaller than image prfcnt, read prfcnt from aimfile ...", 1);
			cmFuncs.searchHeadPositionX64(FLAim);

			if (feof(FLAim))
			{
				continue;
			}
			memset(arAimFrame, sizeof(UINT8), lenAimFrame);
			int fdAimRes = fread(arAimFrame, sizeof(UINT8), lenAimFrame, FLAim);
			if (fdAimRes<lenAimFrame)
			{
				continue;
			}

			prfCntAim = cmFuncs.getPRFNum(arAimFrame);  // modified by luo;

		}
		else  //点迹的PRF计数和图像的PRF计数相同
		{
			match_flag = true;

			muInfo mu0(headerImageAr);
			if (((long(mu0.date_year)) % 100 == 16) && (mu0.date_month == 7))
				loopNum = long(cmFuncs.getLoopNum(headerImageAr) / (picsNum / 8.0));
			else
				loopNum = long(cmFuncs.getLoopNum(headerImageAr));


			//if(NEW_VERSION) loopNum = long(cmFuncs.getLoopNum(headerImageAr)); // modified 2016/8/24

			cmFuncs.logRecords("Target prfcnt is same as image prfcnt, loopNum: ", loopNum);

			UINT8 imgAr[IMAGE_AUX_SIZE];
			memcpy(imgAr, headerImageAr, sizeof(UINT8)*IMAGE_AUX_SIZE);

			string strMergeName = overlayPath + cmFuncs.getMarkPicName(headerImageAr, loopNum) + ".tif";	  // modify by zhang @ 20160224 : string strMergeName=strDest+cmFuncs.getMarkPicName(arAimFrame,loopNum)+".tiff";	
			string strRawName = imagePath + cmFuncs.getRawPicName(headerImageAr, loopNum) + ".tif";

			string tar_map_file = cmFuncs.getMarkPicName(headerImageAr, loopNum) + ".tif";

			for (int k = 1; k<picsNum; k++)
			{
				if (fls.empty())
				{
					break;
				}

				ss = fls.front();
				fls.pop();
				imageFl = flFolder + "\\" + ss;

				WriteProgInfo(ss.c_str()); // write to progress 

				FLImage = fopen(imageFl.c_str(), "rb");
				fread(headerImageAr, sizeof(UINT8), hdLen, FLImage);
				int tempLoop = cmFuncs.getPRFNum(headerImageAr);
				memcpy(lastImageAr, headerImageAr, sizeof(UINT8)*hdLen); // added by luo 
				fdImageRes = fread(&(arImageFrame[k*lenImageFrame]), sizeof(UINT8), lenImageFrame, FLImage);
				fclose(FLImage);
			}

			//
			GMTIModel GMTIMd(arAimFrame, overlayPath);
			GMTIMd.ar_last = lastImageAr;
			GMTIMd.ml2_coef = az_ml2_coef;
			GMTIMd.im_ar = imgAr;	// New added by Yunhua-Luo 2016/12/09

			muInfo mmU(lastImageAr); // modified by luo 
			GMTIMd.lastPlaneLon = mmU.plane_longitude;
			GMTIMd.lastPlaneLat = mmU.plane_latitude;

			/* ==== modify by zhang : add width and height information ==== */
			cmFuncs.logRecords("Output SAR-GMTI merged image information ", 1);
			GMTIMd.overlay = false;
			GMTIMd.rootPath = imagePath;
			GMTIMd.outPutGMTIInfo(loopNum, rgPoints, picsNum * 512, az_ml2_coef);

			GMTIMd.overlay = true;
			GMTIMd.rootPath = overlayPath;
			GMTIMd.outPutGMTIInfo(loopNum, rgPoints, picsNum * 512, az_ml2_coef);

			sarGMTIAimInfo gmtiAim(arAimFrame, lastImageAr, GMTIMd.gps_info, tar_map_file);

			GMTIMd.gmtiAimPt = &gmtiAim;
			int aimNumTemp = gmtiAim.aimsNum;

			long *aimAzPts = new long[aimNumTemp];
			long *aimRgPts = new long[aimNumTemp];

			if (az_ml2_coef <= 1)
			{
				for (int p = 0; p<aimNumTemp; p++)
				{
					aimAzPts[p] = (long)(gmtiAim.Tar_Azloc[p] * az_ml2_coef);
					aimRgPts[p] = (long)gmtiAim.aimRangePoints[p];
				}
			}
			else
			{
				for (int p = 0; p<aimNumTemp; p++)
				{
					aimAzPts[p] = (long)(gmtiAim.Tar_Azloc[p]);
					aimRgPts[p] = (long)(gmtiAim.aimRangePoints[p] / az_ml2_coef);
				}
			}

			int img_height = 512 * picsNum;
			int img_width = rgPoints;
			if (ACCURATE_ML && az_ml2_coef != 1)
			{
				imgProc im;

				if (az_ml2_coef < 1)
					im.ImgScale(arImageFrame, img_width, img_height, 1, az_ml2_coef);
				else
					im.ImgScale(arImageFrame, img_width, img_height, 1.0 / az_ml2_coef, 1);
			}

			// modify by zhang ： change order
			GMTIMd.rootPath = infoPath;
			GMTIMd.img_col = img_width;
			GMTIMd.img_row = img_height;
			//cmFuncs.logRecords("Output SAR-GMTI target datfile ",1);
			GMTIMd.outPutGMTIData();

			GMTIMd.rootPath = overlayPath;

			ImgMark sarGmtiMergeCl;
			sarImageInfo si(arAimFrame);


			int pathLen = strMergeName.length() + 1;
			char * pathMark = new char[pathLen];
			memcpy(pathMark, strMergeName.c_str(), pathLen);

			pathLen = strRawName.length() + 1;
			char * pathRaw = new char[pathLen];
			memcpy(pathRaw, strRawName.c_str(), pathLen);

			sarGmtiMergeCl.SAR_GmtiTar_Mark(pathMark, pathRaw, arImageFrame, aimAzPts, aimRgPts, gmtiAim.Tar_azrev, gmtiAim.aimV, img_width, img_height, gmtiAim.aimsNum, si.look_Side, GMTIMd.img_latis, GMTIMd.img_logns);
			//cmFuncs.logRecords("Merge Complete:",(double)loopNum);

			if (aimAzPts != NULL)
			{
				delete[]aimAzPts;
				aimAzPts = NULL;
			}

			if (aimRgPts != NULL)
			{
				delete[]aimRgPts;
				aimRgPts = NULL;
			}


			//搜索下一个点迹帧
			// for the first time the same as the previous 

			endAimMark = cmFuncs.searchHeadPositionX64(FLAim);
			memset(arAimFrame, sizeof(UINT8), lenAimFrame);
			fdAimRes = fread(arAimFrame, sizeof(UINT8), lenAimFrame, FLAim);

			if (fdAimRes != lenAimFrame) break;  // add by zhang : FLAim missing frame

			prfCntAim = cmFuncs.getPRFNum(arAimFrame);

			if (fls.empty())
			{
				break;
			}
			ss = fls.front();
			fls.pop();
			imageFl = flFolder + "\\" + ss;
			FLImage = fopen(imageFl.c_str(), "rb");

			memset(headerImageAr, sizeof(UINT8), hdLen);
			fread(headerImageAr, sizeof(UINT8), hdLen, FLImage);

			prfCntImage = cmFuncs.getPRFNum(headerImageAr);
			rgPoints = cmFuncs.getRangePoints(headerImageAr);
			lenImageFrame = rgPoints * 512;
			imageArSize = lenImageFrame*picsNum;

			if (arImageFrame != NULL)
			{
				delete[] arImageFrame;
				arImageFrame = NULL;
			}
			arImageFrame = new UINT8[imageArSize];
			memset(arImageFrame, sizeof(UINT8), imageArSize);
			int fdImageRes = fread(arImageFrame, sizeof(UINT8), lenImageFrame, FLImage);
			fclose(FLImage);

			if (fls.empty())
			{
				continue;
			}

		}//end if

	} while (!fls.empty());


	if (arAimFrame != NULL)
	{
		delete[] arAimFrame;
		arAimFrame = NULL;
	}

	if (lastImageAr != NULL)
	{
		delete[] lastImageAr;
		lastImageAr = NULL;
	}

	if (headerImageAr != NULL)
	{
		delete[] headerImageAr;
		headerImageAr = NULL;
	}

	fclose(FLAim);

	WriteProgInfo("end_of_proc"); // write to progress 

	if (!match_flag) cmFuncs.logRecords("Can not find matched aimfile and image files ", 1);

	return;

}

// ***** 同时SAR/GMTI模式实时处理子函数v2.0 **********
int sarGMTIInfoRealMergeGai(UINT8 *data, __int64 dataLen, char *szDstPath, char *szTrackFilePath)
{
	// FIRST CHECK FINISHED 2016/11/2 14:50;
	// SECOND CHECK FINISHED 2016/11/2 20:00

	// Check data 
	commonFuncs cmFuncs;

	char szTrackFile[512];
	strcpy(szTrackFile, szTrackFilePath);
	int len1 = strlen(szTrackFilePath);
	sprintf(szTrackFile + len1, "DZS_GMTI.DAT");

	cmFuncs.logRecords("  Entry into real-time SAR/GMTI ... ", dataLen);
	cmFuncs.logRecords(szTrackFile, 0);
	cmFuncs.logRecords(szDstPath, 0);

	if (cmFuncs.getHeadPosition(data, dataLen) != 0)
	{
		cmFuncs.logRecords("  sar gmti real image data format error", 0);
		return 0;
	}

	if (cmFuncs.getSarModel(data) != 3)
	{
		cmFuncs.logRecords("  image data mode error", 0);
		return 0;
	}

	int TRACK_DATA_SIZE = BYTE_PER_TAR_MTI * 8192;
	TRACK_DATA_SIZE = BYTE_PER_TAR_MTI * 3072;
	__int64 TRACK_SIZE = TRACK_AUX_SIZE + TRACK_DATA_SIZE;
	__int64 IMAGE_BUFSIZE = __int64(MAX_IMAGE_LEN)*__int64(IMAGE_BUFFER_SIZE);
	__int64 TRACK_BUFSIZE = __int64(TRACK_SIZE)*__int64(TRACK_BUFFER_SIZE);

	static BYTE *imageBuffer = NULL;
	static BYTE *trackBuffer = NULL;

	static int lastPowerOnTime = -1;
	static int loopNum = 0;				// TODO : to be read from aux
	static int nImgBufCnt;
	static long nCnt = 0;
	static float az_ml2_coef = 1;
	nCnt++;

	int currentPowerOnTime = cmFuncs.getPowerOnTime(data);
	loopNum = cmFuncs.getLoopNum(data);

	char imagePath[512];
	char infoPath[512];
	char overlayPath[512];

	/* create folder 'SAR-GMTI' */

	char *outputPath = (char *)malloc(512);
	ZeroMemory(outputPath, 512);
	int len = strlen(szDstPath);
	CopyMemory(outputPath, szDstPath, len);
	if (outputPath[len - 1] == '\\')
	{
		len += sprintf(outputPath + len, "SAR-GMTI\\");
	}
	else {
		len += sprintf(outputPath + len, "\\SAR-GMTI\\");
	}
	_mkdir((const char *)outputPath);

	/* create sub folder 'SAR-GMTI\TUXIANG' */

	CopyMemory(imagePath, outputPath, 512);
	sprintf(imagePath + len, "TUXIANG\\");
	_mkdir((const char *)imagePath);

	/* create sub folder 'SAR-GMTI\DIANJI' */
	CopyMemory(infoPath, outputPath, 512);
	sprintf(infoPath + len, "DIANJI\\");
	_mkdir((const char *)infoPath);

	/* create sub folder 'SAR-GMTI\DIEJIATU' */
	CopyMemory(overlayPath, outputPath, 512);
	sprintf(overlayPath + len, "DIEJIATU\\");
	_mkdir((const char *)overlayPath);
	free(outputPath);

	if (nCnt == 1 || (currentPowerOnTime != lastPowerOnTime))  // first come in to clear all 
	{
		if (ACCURATE_ML)
		{
			sarImageInfo img_info0(data);
			az_ml2_coef = img_info0.ml2_coef;
		}

		nImgBufCnt = 0;
		lastPowerOnTime = currentPowerOnTime;
		loopNum = 0;
		if (imageBuffer != NULL) delete[] imageBuffer; imageBuffer = NULL;
		if (trackBuffer != NULL) delete[] trackBuffer; trackBuffer = NULL;

	}
	lastPowerOnTime = currentPowerOnTime;

	// allocate buffer 
	if (imageBuffer == NULL)
	{
		imageBuffer = new BYTE[IMAGE_BUFSIZE];
		if (imageBuffer == NULL) { cmFuncs.logRecords("ImageBuffer allocate failed", nCnt); return 0; }
		memset(imageBuffer, 0, sizeof(BYTE)*IMAGE_BUFSIZE);
		nImgBufCnt = 0;
	}

	if (trackBuffer == NULL)
	{
		trackBuffer = new BYTE[TRACK_BUFSIZE];
		if (trackBuffer == NULL) { cmFuncs.logRecords("trackBuffer allocate failed", nCnt); return 0; }
		memset(trackBuffer, 0, TRACK_BUFSIZE);
	}


	// ***************** STEP.1 Prepared all data ********************

	// store image data into buffer
	// TODO : dataLen should not change among each small picture
	// TODO : check overlap of nImgBufTop and nImgBufBtm

	cmFuncs.logRecords("  copy image data to buffer ... ", nImgBufCnt);
	memcpy(imageBuffer + __int64(MAX_IMAGE_LEN)*__int64(nImgBufCnt), data, sizeof(BYTE)*dataLen);
	nImgBufCnt++;
	nImgBufCnt = nImgBufCnt % IMAGE_BUFFER_SIZE;

	cmFuncs.logRecords("  copy image data to buffer finished ", nImgBufCnt);

	// Get lastest valid tracks into trackBuffer 
	int TrackNum = GetTrackDataFromFile(szTrackFile, trackBuffer);
	if (TrackNum < 0) return 0;
	cmFuncs.logRecords("  Read track finished : ", TrackNum);

	//long track_prf  = cmFuncs.getPRFNum(trackBuffer);
	//long imge_prf   = cmFuncs.getPRFNum(imageBuffer+MAX_IMAGE_LEN*(nImgBufCnt-1));
	//cmFuncs.logRecords("  Track prf : ", track_prf);
	//cmFuncs.logRecords("  Image prf : ", imge_prf);

	// *********** Step.2 Match Track data and image data *************
	__int64 trackPRF = 0;
	__int64 imagePRF = 0;

	int m, n;
	BYTE* pImage;
	BYTE* pTrackBuf;
	bool  beMatched = false;
	for (n = 0; n < TRACK_BUFFER_SIZE; n++)
	{
		if (beMatched) break;

		pTrackBuf = trackBuffer + n*TRACK_SIZE;
		if ((*pTrackBuf) == 0) continue;
		trackPRF = cmFuncs.getPRFNum(pTrackBuf);
		if (trackPRF == 0) continue;  // alway the lastest track in the front;

		for (m = 0; m < IMAGE_BUFFER_SIZE; m++)
		{
			pImage = imageBuffer + m*MAX_IMAGE_LEN;
			imagePRF = cmFuncs.getPRFNum(pImage);
			if ((*pImage) == 0 || imagePRF == 0) continue;  // empty slot

			if (imagePRF == trackPRF && imagePRF != 0)
			{
				beMatched = true; break; // break 只能跳出单层循环
			}
			else  continue;

		}
	}

	if (!beMatched)
	{
		cmFuncs.logRecords(" image prf and track prf dismatched ... ", nImgBufCnt);
		return 0;
	}

	cmFuncs.logRecords(" matched image no:     ", m);
	cmFuncs.logRecords(" matched track prf:    ", trackPRF);

	// CHECK VALID RAW FILES 
	int nView = cmFuncs.getMViewNum(pImage);
	int nPicNum = 64 / nView;
	cmFuncs.logRecords("Small pic num: ", nPicNum);

	BYTE* pTemp;
	int pos;
	for (int k = m + 1; k<m + nPicNum; k++)
	{
		pos = k % IMAGE_BUFFER_SIZE;
		pTemp = imageBuffer + pos*MAX_IMAGE_LEN;
		if ((*pTemp) == 0 || cmFuncs.getPRFNum(pTemp) < imagePRF)
		{
			cmFuncs.logRecords("Not enough small raw files", pos);
			return 0;
		}
	}

	cmFuncs.logRecords("image and track prf matched ... ", trackPRF);
	// ************ Step.3 Output image and merged image is ok **************
	BYTE lastArData[IMAGE_AUX_SIZE];
	BYTE auxBuf[IMAGE_AUX_SIZE];

	memcpy(auxBuf, pImage, IMAGE_AUX_SIZE);  // first raw file 

											 // pick up small raw data
	long subImgSize = dataLen - IMAGE_AUX_SIZE;
	BYTE *mergeBuffer = new BYTE[nPicNum * subImgSize]; //(BYTE *)malloc(nPicNum * subImgSize);
	if (mergeBuffer == NULL) { cmFuncs.logRecords("MergeBuffer allocate failed ...", 0); return 0; }
	for (int k = 0; k < nPicNum; k++)
	{
		int nSmallPic = PickupSmallPic(imageBuffer, IMAGE_BUFFER_SIZE, trackPRF + k * 512 * nView);
		cmFuncs.logRecords("Find prfcnt matched pic no in the image buffer: ", nSmallPic);

		if (nSmallPic < 0)
		{  // current small picture lost
			memset(mergeBuffer + k*subImgSize, 0, sizeof(BYTE)*subImgSize);
		}
		else
		{
			memcpy(mergeBuffer + k * subImgSize, imageBuffer + nSmallPic*MAX_IMAGE_LEN + IMAGE_AUX_SIZE, sizeof(BYTE)*subImgSize);
			memcpy(lastArData, imageBuffer + nSmallPic*MAX_IMAGE_LEN, sizeof(BYTE)*IMAGE_AUX_SIZE);
			memset(imageBuffer + nSmallPic*MAX_IMAGE_LEN, 0, MAX_IMAGE_LEN);
		}
	}

	cmFuncs.logRecords("  output merge image and txt info  ... ", 0);

	muInfo mu0(auxBuf);
	if ((long(mu0.date_year) % 100 == 16) && (mu0.date_month == 7))
		loopNum = long(cmFuncs.getLoopNum(auxBuf) / (nPicNum / 8.0));
	else
		loopNum = long(cmFuncs.getLoopNum(auxBuf));

	// get track data
	GMTIModel GMTIMd(pTrackBuf, overlayPath);
	GMTIMd.ar_last = lastArData;  // used for location 
	GMTIMd.im_ar = auxBuf;  // 只有图像RAW辅助数据里面有压缩比，点迹文件没有；

	muInfo mmU(lastArData);
	GMTIMd.lastPlaneLon = mmU.plane_longitude;
	GMTIMd.lastPlaneLat = mmU.plane_latitude;

	int rgPoints = cmFuncs.getRangePoints(auxBuf);

	cmFuncs.logRecords("  Output SAR-GMTI image information", loopNum);
	GMTIMd.overlay = false;
	GMTIMd.rootPath = imagePath;
	GMTIMd.outPutGMTIInfo(loopNum, rgPoints, nPicNum * 512, az_ml2_coef);

	GMTIMd.overlay = true;
	GMTIMd.rootPath = overlayPath;
	GMTIMd.outPutGMTIInfo(loopNum, rgPoints, nPicNum * 512, az_ml2_coef);

	string szMergeName = overlayPath + cmFuncs.getMarkPicName(auxBuf, loopNum) + ".tif";
	string szRawName = imagePath + cmFuncs.getRawPicName(auxBuf, loopNum) + ".tif";

	string tar_map_file = cmFuncs.getMarkPicName(pTrackBuf, loopNum) + ".tif";  // 没有压缩方式001

	sarGMTIAimInfo gmtiAim(pTrackBuf, lastArData, GMTIMd.gps_info, tar_map_file);

	GMTIMd.gmtiAimPt = &gmtiAim;
	int aimNumTemp = gmtiAim.aimsNum;
	GMTIMd.ml2_coef = az_ml2_coef;

	long *aimAzPts = new long[aimNumTemp];
	long *aimRgPts = new long[aimNumTemp];

	if (az_ml2_coef <= 1)
	{
		for (int p = 0; p < aimNumTemp; p++)
		{
			aimAzPts[p] = (long)(gmtiAim.Tar_Azloc[p] * az_ml2_coef);
			aimRgPts[p] = (long)gmtiAim.aimRangePoints[p];
		}
	}
	else
	{
		for (int p = 0; p < aimNumTemp; p++)
		{
			aimAzPts[p] = (long)(gmtiAim.Tar_Azloc[p]);
			aimRgPts[p] = (long)(gmtiAim.aimRangePoints[p] / az_ml2_coef);
		}
	}

	// Image Scaling 

	int img_height = 512 * nPicNum;
	int img_width = rgPoints;
	if (ACCURATE_ML && az_ml2_coef != 1)
	{
		imgProc im;
		if (az_ml2_coef < 1)
			im.ImgScale(mergeBuffer, img_width, img_height, 1, az_ml2_coef);
		else
			im.ImgScale(mergeBuffer, img_width, img_height, 1.0 / az_ml2_coef, 1);
	}

	cmFuncs.logRecords("  Send SAR-GMTI track to E-MAP", loopNum);
	GMTIMd.rootPath = infoPath;
	GMTIMd.img_col = img_width;
	GMTIMd.img_row = img_height;
	GMTIMd.outPutGMTIData();

	cmFuncs.logRecords("  Output merge image start", loopNum); // 存在数据奔馈点
	GMTIMd.rootPath = overlayPath;
	sarImageInfo si(auxBuf);
	ImgMark sarGmtiMergeCl;
	sarGmtiMergeCl.SAR_GmtiTar_Mark((char *)szMergeName.c_str(), (char *)szRawName.c_str(), mergeBuffer, aimAzPts, aimRgPts, gmtiAim.Tar_azrev, gmtiAim.aimV, img_width, img_height, gmtiAim.aimsNum, si.look_Side, GMTIMd.img_latis, GMTIMd.img_logns);

	delete[] aimAzPts; aimAzPts = NULL;
	delete[] aimRgPts; aimRgPts = NULL;
	delete[] mergeBuffer; mergeBuffer = NULL;

	memset(pTrackBuf, 0, TRACK_SIZE);

	return 0;
}

// *******  WAS-GMTI 处理功能函数  ***************
int WasGMTIDatProc(UINT8 *dataAr, const char *strDest, int len)
{
	commonFuncs cmFuncs;

	static long burst_cnt = 0;

	cmFuncs.logRecords("WAS-MTI Burst No : ", burst_cnt);

	//Sleep(1000);

	GY_GMTIModel Gy_GMTIMd(dataAr, strDest);

	Gy_GMTIMd.Generate_WasMTIAim();

	Gy_GMTIMd.Generate_WasMTITrack();

	burst_cnt++;

	return 0;
}

// ********** SAR成像模式处理子函数 ***************
int creatTIFF(UINT8 *RdataAr, const char *strDest, __int64 len)
{
	commonFuncs cmFuncs;
	//cmFuncs.logRecords("Entry CreatTIFF Function() for image mode ...",int(len));

	static long i = 0;
	i++;

	//方位向号码
	static int loopNum = 0;
	static int pwOnTime = 1;
	static int workmode_prev = -1;
	static unsigned __int64 prf_count_prev = 0;

	//发现数组经过Mat处理之后,出现变化。
	__int64 shRes = cmFuncs.getHeadPosition(RdataAr, len);

	if (shRes < 0)
	{
		cmFuncs.logRecords("Guidecode Error ... ", double(shRes));
		return ERROR_Search;
	}

	if (shRes > 0)
	{
		cmFuncs.logRecords("Guidecode location ... : ", double(shRes));
	}

	UINT8   *dataAr = RdataAr + shRes;
	__int64 data_len = len - shRes;

	__int64 rgPoints = cmFuncs.getRangePoints(dataAr);

	if (rgPoints<512)
	{
		cmFuncs.logRecords("Error in tiff Range Points", 1);
		return ERROR_TIFF;
	}

	UINT32 lenFrame = rgPoints*HEADLEN + 512;

	string rtPath(strDest);
	string mergePath(strDest);
	int rtRes = 0;
	char chMgFolder[512];

	/***************************************/
	int mdJudge = cmFuncs.getSarModel(dataAr);

	if (mdJudge < 3)  // SAR-IMAGE mode  0 - spotlight 1 - slidespot 2 - strip
	{
		rtPath.append("SAR-IMAGE\\");
		mergePath.append("SAR-IMAGE\\");
		_mkdir(rtPath.c_str());

	}
	else // other modes 
	{
		return -1;

	}

	/* ==== add by zhang : reset power on time ==== */
	static int lastPowerOnTime = -1;
	int currentPowerOnTime = cmFuncs.getPowerOnTime(dataAr);

	if (currentPowerOnTime != lastPowerOnTime)
	{
		i = 1;		// for new image with new az_ml2_coef value ;
	}

	if (workmode_prev != mdJudge)
	{
		i = 1;
	}
	workmode_prev = mdJudge;

	UINT64 prf_count_cur = cmFuncs.getPRFNum(dataAr);
	if (abs(long(prf_count_cur - prf_count_prev))>5000)  // PRF跳跃太多；
	{
		i = 1;
	}
	prf_count_prev = prf_count_cur;
	loopNum = cmFuncs.getLoopNum(dataAr); // new version 
	lastPowerOnTime = currentPowerOnTime;

	// Calculate multilook coef 
	static float az_ml2_coef = 1.0;
	if (ACCURATE_ML && i == 1)
	{
		sarImageInfo img_info0(dataAr);
		az_ml2_coef = img_info0.ml2_coef;
	}

	// 图像拼接操作
	imgProc procImg(dataAr, rtPath);
	procImg.az_ml2_coef = az_ml2_coef;

	procImg.creatCoTif_New(dataAr, data_len);			// modified by Luo 20121/07/12

	return rtRes;
}

// *** SAR-IMAGE 实时多块内存图像处理函数,V2022/08/19 ********
int creatTIFF_Online(UINT8 *dataAr, const char *strDest, __int64 len, UINT8 blocknum)
{
	UINT8 kk;
	UINT8 flag = 1;
	for (kk = 0; kk < blocknum; kk++)
	{
		flag = creatTIFF(dataAr + len*kk, strDest, len);
	}

	return flag;
}





// **************************************************************************
// ********************* 实时和离线处理入口主函数 ***************************

// 离线处理入口函数

// *******  WAS-GMTI模式离线还原处理入口函数V2020 *********
extern "C" _declspec(dllexport) int _stdcall WasGMTIOfflineProc(const char *strSource, const char *strDest)
{
	commonFuncs cmFuncs;
	static int iiCnt = 0;

	memset(DllPath, 0, sizeof(char) * 1024);
	::GetModuleFileNameA(NULL, (LPSTR)DllPath, sizeof(DllPath));
	string str0(DllPath);
	int pos = str0.find_last_of("\\");
	str0.replace(pos, str0.length() - pos, "\\offline_prog.txt");
	memset(DllPath, 0, sizeof(char) * 1024);
	strcpy(DllPath, str0.c_str());
	cmFuncs.logRecords(DllPath, 0.0);

	remove(DllPath);
	WriteProgInfo(""); // write to progress

	int hdLen = 1024;				//SAR-GMTI和广域GMTI的点迹帧头有1024个字节
	string rtStr(strDest);
	rtStr.append("GY_MTI\\");
	_mkdir(rtStr.c_str());

	UINT32 step = 1024 * 64 * 512;	//搜索步长，可以根据需要定义

									//实际帧长度
	UINT32  lenFrame = 0;
	FILE    *FL = fopen(strSource, "rb");
	__int64 pp = ftell(FL);

	//******* step.1  查找第一个特征码 01 DC EF 18 ***********
	UINT8 *searchBuffer = new UINT8[step];
	UINT8 *headerAr = new UINT8[hdLen];
	UINT8 *arFrame = new UINT8[17920];
	int endMark = feof(FL);

	while (endMark == 0)
	{
		UINT32 rdState = fread(searchBuffer, sizeof(UINT8), step, FL);

		if (rdState<hdLen)
		{
			return 0;
		}

		int re = cmFuncs.getHeadPosition(searchBuffer, rdState);

		if (re != -1) // 找到帧头
		{
			fseek(FL, re - rdState, SEEK_CUR);	// 指针移位到01DCEF18
			break;
		}
		else
		{
			//防止帧头标志位处于两帧分割之间
			fseek(FL, -1 * HD_LEN, SEEK_CUR);
		}
	}

	//******* step.2  循环读取数据 ***********

	while (endMark == 0)
	{
		//去除不满一帧的情况
		int fdRes = fread(headerAr, sizeof(UINT8), hdLen, FL);

		// added by luo
		int flag_cnt = int(headerAr[0] == 1) + int(headerAr[1] == 220) + int(headerAr[2] == 239) + int(headerAr[3] == 24);
		if (flag_cnt < 3) cmFuncs.logRecords("Guidecode Error ... ", 4 - flag_cnt);

		//把文件指针放在帧头标志位处
		fseek(FL, -1 * hdLen, SEEK_CUR);

		if (fdRes<hdLen)//读取值小于帧头长度
		{
			return ERROR_Search;
		}

		int md = cmFuncs.getSarModel(headerAr);

		if (md == 4)	// WAS-GMTI 
		{

			int aimsNumIntTemp = cmFuncs.getNewGYAimsNum(headerAr);

			/*
			if(aimsNumIntTemp<=0)  continue; // can not be equal zero
			int aimsParsLen=0;
			if((aimsNumIntTemp*BYTE_PER_TAR_WAS)%512 != 0)
			{
			aimsParsLen = ((int)((aimsNumIntTemp*BYTE_PER_TAR_WAS)/512)+1)*512;
			}
			else
			{
			aimsParsLen=aimsNumIntTemp*BYTE_PER_TAR_WAS;
			}
			*/
			lenFrame = 17920; // for V4 20210121 ; // aimsParsLen+1536; // modified by luo 20190611

			fdRes = fread(arFrame, sizeof(UINT8), lenFrame, FL);

			if (fdRes<lenFrame) // file end 
			{
				break;
			}

		}
		else // 其他模式不处理
		{
			continue;
		}

		iiCnt++;
		WasGMTIDatProc(arFrame, rtStr.c_str(), lenFrame);   // 处理的核心代码

		cmFuncs.searchHeadPosition(FL);
		endMark = feof(FL);

	}

	// *********************************************
	if (searchBuffer != NULL)
	{
		delete[]searchBuffer;
		searchBuffer = NULL;
	}

	if (headerAr != NULL)
	{
		delete[]headerAr;
		headerAr = NULL;
	}

	cmFuncs.logRecords("Total WAS-GMTI frame number : ", iiCnt);
	fclose(FL);

	WriteProgInfo("end_of_proc"); // write to progress 

	return 0;
}


// ******* SAR-IMAGE mode and SAR-GMTI mode 离线处理函数入口函数 V2020  ********** 
extern "C" _declspec(dllexport) int _stdcall ImageModeOfflineProc(const char *strImageFolder, const char *strDest)
{
	// 搜索目录
	commonFuncs cmFuncs;

	memset(DllPath, 0, sizeof(char) * 1024);
	::GetModuleFileNameA(NULL, (LPSTR)DllPath, sizeof(DllPath));
	string str0(DllPath);
	int pos = str0.find_last_of("\\");
	str0.replace(pos, str0.length() - pos, "\\offline_prog.txt");
	memset(DllPath, 0, sizeof(char) * 1024);
	strcpy(DllPath, str0.c_str());
	cmFuncs.logRecords(DllPath, 0.0);

	remove(DllPath);

	WIN32_FIND_DATA FindData;
	HANDLE hError;

	char FilePathName[1024];
	strcpy(FilePathName, strImageFolder);
	strcat(FilePathName, "\\*.raw");		// modified by luo yunhua 
	string FileFoderPath(strImageFolder);

	hError = ::FindFirstFile(FilePathName, &FindData);
	if (hError == INVALID_HANDLE_VALUE)
	{
		cmFuncs.logRecords("can not find raw image file in folder ... ", 0.0);
		return ERROR_PATH;
	}

	int mdJudge = 0;
	string str(FindData.cFileName);

	if (str.size()>4)
	{
		const UINT32 LENCONST = 5120 * 32768;

		string strImgFile;
		strImgFile = FileFoderPath + str;

		FILE *FLImage = fopen(strImgFile.c_str(), "rb");

		if (FLImage == NULL) return ERROR_PATH;

		UINT8 *chAr = new UINT8[LENCONST];
		memset(chAr, sizeof(UINT8), LENCONST);
		int rdLen = fread(chAr, sizeof(UINT8), LENCONST, FLImage);
		fclose(FLImage);

		int shRes = cmFuncs.getHeadPosition(chAr, rdLen);

		//帧格式错误
		if (shRes != 0)
		{
			cmFuncs.logRecords("Can not find guidecode in raw data ", chAr[0]);
			return ERROR_Search;
		}

		mdJudge = cmFuncs.getSarModel(chAr);

		delete[] chAr;
		chAr = NULL;

	}
	else
	{
		cmFuncs.logRecords("Can not find raw image in diretory ", 0);
		return ERROR_Search;
	}


	if (mdJudge == 3) // SAR-GMTI
	{
		char strAimPath[1024];
		strcpy(strAimPath, strImageFolder);
		strcat(strAimPath, "\\*.dat");     // modified by luo yunhua 

		hError = ::FindFirstFile(strAimPath, &FindData);
		if (hError == INVALID_HANDLE_VALUE)
		{
			cmFuncs.logRecords("can not find aim path file in folder ... ", 0.0);
			return ERROR_PATH;
		}

		string AimFile(FindData.cFileName);

		AimFile = FileFoderPath + AimFile; // added by luo ;

		sarGMTIOfflineProc(strImageFolder, AimFile.c_str(), strDest);

	}
	else if (mdJudge < 3) // SAR-Mode
	{
		// SAR-Mode : get all raw data and process one by one 
		char strRawPath[1024];
		strcpy(strRawPath, strImageFolder);
		strcat(strRawPath, "\\*.raw");     // modified by luo yunhua 

		hError = ::FindFirstFile(strRawPath, &FindData);
		if (hError == INVALID_HANDLE_VALUE)
		{
			cmFuncs.logRecords("can not find raw image file in folder ... ", 0.0);
			return ERROR_PATH;
		}

		const UINT32 LENCONST = 5120 * 32768;
		UINT8 *data = new UINT8[LENCONST];
		memset(data, sizeof(UINT8), LENCONST);
		FILE *FLImage;

		while (1)
		{
			string str(FindData.cFileName);

			if (str.size()>4)
			{
				WriteProgInfo(str.c_str()); // write to progress 

											// Read data file and processsed one by one 
				str = FileFoderPath + str;

				FLImage = fopen(str.c_str(), "rb");
				if (FLImage == NULL) continue;

				memset(data, sizeof(UINT8), LENCONST);
				int rdLen = fread(data, sizeof(UINT8), LENCONST, FLImage);
				fclose(FLImage);

				creatTIFF(data, strDest, rdLen);

			}

			if (!(::FindNextFile(hError, &FindData))) break;
		}

		WriteProgInfo("end_of_proc"); // write to progress 

		delete[] data;
		data = NULL;
	}

}


extern "C" _declspec(dllexport) int _stdcall ImageModeOnlineProc_GAI(UINT8 *data, __int64 dataLen, UINT8 blocknum, char *szDstPath, char *szTrackFile);
// data		   --- 图像内存起始地址；
// datalen	   --- 单个RAW图像大小；
// blocknum    --- 每次内存RAW图像块数
// szDstPath   --- 输出目录
// szTrackFile --- 实时点迹文件路径



// 实时处理模式调用函数 CHECK BY LUO 2021/07/14

// ******* SAR-IMAGE mode and SAR-GMTI mode 实时处理函数入口函数 V2020/03/04 ok **********
extern "C" _declspec(dllexport) int _stdcall ImageModeOnlineProc(UINT8 *data, __int64 dataLen, char *szDstPath, char *szTrackFile)
{

	commonFuncs cmFuncs;
	cmFuncs.logRecords("Entry ImageModeOnLineProc() ...", int(dataLen));

	//发现数组经过MAT处理之后,出现变化。
	int shRes = cmFuncs.getHeadPosition(data, dataLen);

	cmFuncs.logRecords("transType: ", data[460]);
	cmFuncs.logRecords("imCompressRatio: ", data[461]);
	cmFuncs.logRecords("imCompressRatio: ", data[462]);

	//帧格式错误
	if (shRes != 0)
	{
		cmFuncs.logRecords("Can not find guidecode in data ", data[0]);
		return ERROR_Search;
	}

	int mdJudge = cmFuncs.getSarModel(data);

	// WAS-GMTI mode 
	if (mdJudge >= 4)
	{
		cmFuncs.logRecords("Wrong work mode ... ", mdJudge);
		return ERROR_Search;
	}

	// SAR-GMTI mode 
	if (mdJudge == 3) // SAR-GMTI
	{
		// Main functions by Luo 
		cmFuncs.logRecords("Entry SAR-GMTI RT mode ... ", 3);
		sarGMTIInfoRealMergeGai(data, dataLen, szDstPath, szTrackFile);
	}

	// SAR-IMAGE mode 
	if (mdJudge < 3)
	{
		cmFuncs.logRecords("Entry SAR-IMAGE RT mode ... ", mdJudge);
		creatTIFF(data, szDstPath, dataLen);
	}

}


extern "C" _declspec(dllexport) int _stdcall ImageModeOnlineProc_GAI(UINT8 *data, __int64 dataLen, UINT8 blocknum, char *szDstPath, char *szTrackFile)
{

	commonFuncs cmFuncs;
	cmFuncs.logRecords("Entry ImageModeOnLineProc() ...", int(dataLen));

	//发现数组经过MAT处理之后,出现变化。
	int shRes = cmFuncs.getHeadPosition(data, dataLen);

	cmFuncs.logRecords("transType: ", data[460]);
	cmFuncs.logRecords("imCompressRatio: ", data[461]);
	cmFuncs.logRecords("imCompressRatio: ", data[462]);

	//帧格式错误
	if (shRes != 0)
	{
		cmFuncs.logRecords("Can not find guidecode in data ", data[0]);
		return ERROR_Search;
	}

	int mdJudge = cmFuncs.getSarModel(data);

	// WAS-GMTI mode 
	if (mdJudge >= 4)
	{
		cmFuncs.logRecords("Wrong work mode ... ", mdJudge);
		return ERROR_Search;
	}

	// SAR-GMTI mode 
	if (mdJudge == 3) // SAR-GMTI
	{
		// Main functions by Luo 
		cmFuncs.logRecords("Entry SAR-GMTI RT mode ... ", 3);
		sarGMTIInfoRealMergeGai(data, dataLen, szDstPath, szTrackFile);
	}

	// SAR-IMAGE mode 
	if (mdJudge < 3)
	{
		cmFuncs.logRecords("Entry SAR-IMAGE RT mode ... ", mdJudge);
		creatTIFF_Online(data, szDstPath, dataLen, blocknum);
	}

}

// ******* WAS-GMTI实时点迹还原主函数入口 2021/07/12 for V4 FPGA sea mode ************
extern "C" _declspec(dllexport) int _stdcall WasMTIOnlineProc(UINT8 *dataAr, const char *strDest, __int64 len)
{// check by luo 2021/07/12 ok

	commonFuncs cmFuncs;

	int shRes = cmFuncs.getHeadPosition(dataAr, len);

	//帧格式错误
	if (shRes != 0)
	{
		cmFuncs.logRecords("Guidecode Error ... ", shRes);
		return ERROR_Search;
	}

	int mdNum = cmFuncs.getSarModel(dataAr);

	float res = cmFuncs.getSarRes(dataAr);

	if (mdNum<0 || mdNum>4)
	{
		return ERROR_MODEL;
	}

	if (mdNum == 3)		// SAR/GMTI mode 
	{
		string rtPath(strDest);
		rtPath.append("SAR-GMTI\\");
		_mkdir(rtPath.c_str());
		rtPath.append("DIANJI\\");
		_mkdir(rtPath.c_str());

	}
	else if (mdNum == 4)  // WAS-GMTI mode 
	{
		string rtPath(strDest);
		rtPath.append("GY-MTI\\");
		_mkdir(rtPath.c_str());

		WasGMTIDatProc(dataAr, rtPath.c_str(), len);
	}
	else
	{
		return ERROR_MODEL;
	}

	return 0;
}







// 预处理软件调用函数 OK

// ********** (9). 预处理软件图像灰度和校正操作函数 check by luo 2016/06/07 ****************

extern "C" _declspec(dllexport) void _stdcall imagePreProcOut(char *strImage, char *strDest, imageProcParsST procPars)
{
	commonFuncs cmFuncs;
	imgProc imageProc;
	imageProc.allImageProMethod(strImage, strDest, procPars);
}


// ********** (10). 预处理软件点迹DAT文件生成TXT函数 check by luo 2016/06/07  *********
extern "C" _declspec(dllexport) void _stdcall gmti6SuoDatRestore(char *strSource, char *strDest, stGMTIProcContent prcCont)
{
	commonFuncs cmFuncs;

	CreateDirectory(strDest, NULL);

	if (prcCont.blSingle)
		cmFuncs.ReadXL_EMapFile_mode_1(strSource, strDest);

	if (prcCont.blMultiple)
		cmFuncs.ReadXL_EMapFile_mode_2(strSource, strDest);

	if (prcCont.blHjtxt)
		cmFuncs.ReadXL_EMapFile_mode_3(strSource, strDest);

}


// ********** (11) 图像拼接 *************************
extern "C" _declspec(dllexport) void _stdcall imageMerge(string *strSource, char *strDest, int N)
{
	int i, j, k;

	Mat src;
	double adfGeoTransform[12];
	long heightI = 8192;
	long widthI = 32768;
	UINT8 *dataR = new UINT8[heightI*widthI];
	memset(dataR, 0, sizeof(UINT8)*heightI*widthI);

	// Read first data for information 
	if (GEOTIFF_VERSION)
	{
		// input strImage is geo-tiff image 
		GeoTiff geo;
		long info[2];
		geo.Read2Geotiff(strSource[0].c_str(), dataR, info, adfGeoTransform);

		widthI = info[0];
		heightI = info[1];

		if (dataR[0] = 0 || widthI < 0)
		{
			printf("image is not geo-tiff format !!!\n");
			return;
		}

	}
	else
	{
		src = imread(strSource[0], -1);
		widthI = src.cols;
		heightI = src.rows;
	}
	delete[] dataR; dataR = NULL;

	__int64 BUF_SIZE = __int64(widthI)*heightI*N;
	UINT8* merge_image = new UINT8[BUF_SIZE];
	if (merge_image == NULL) return;

	long new_height = N*long(heightI);

	for (i = 0; i<N; i++)
	{

		if (GEOTIFF_VERSION)
		{
			// input strImage is geo-tiff image 
			GeoTiff geo;
			long info[2];
			geo.Read2Geotiff(strSource[i].c_str(), merge_image, info, adfGeoTransform);

			widthI = info[0];
			heightI = info[1];

			if (merge_image[0] = 0 || widthI < 0)
			{
				printf("image is not geo-tiff format !!!\n");
				return;
			}

		}
		else
		{
			src = imread(strSource[i].c_str(), -1);
			widthI = src.cols;
			heightI = src.rows;
			memcpy(merge_image, src.data, sizeof(UINT8)*heightI*widthI);
		}

		// ******************
		long data_size = __int64(widthI)*heightI;
		memcpy(merge_image + (N - 1 - i)*data_size, merge_image, sizeof(UINT8)*data_size);

	}

	if (GEOTIFF_VERSION)
	{
		GeoTiff geo;
		char *gcs_wgs1984 = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.01745329251994328,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]]";
		geo.GeotiffWriteFromByteData(merge_image, new_height, widthI, strDest, adfGeoTransform, gcs_wgs1984); //调用下面的写geotiff函数

	}
	else
	{
		src.data = merge_image;
		src.cols = widthI;
		src.rows = new_height;
		imwrite(strDest, src);
	}

	delete[] merge_image; merge_image = NULL;

}