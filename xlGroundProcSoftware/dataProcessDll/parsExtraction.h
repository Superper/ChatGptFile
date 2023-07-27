// ********** XX ����ʵʱ������滹ԭ�� V4.0 ***********
	//        Dev by Yunhua-Luo @2016/8/11 - 2020/03/04
	//        New modified by Yunhua-Luo 20181109
	//        Updated by Yunhua-Luo 20190504 
	//        Updated by Yunhua-Luo 20200304
// updated by Luoyunhua 20201124
//  **********************************************

#include "stdafx.h"
#include "OpenCV_HeaderFile.h"
#include "dataProcessDll.h"
#include <winsock2.h>
#include <ws2tcpip.h>
#include <direct.h>
#include <string.h>
#include <queue>
#include <omp.h>
#include <Windows.h>
#include <fstream>
#include <math.h>
#include "MulTarTrk.h"

#define UINT8  unsigned __int8
#define UINT16 unsigned __int16
#define UINT32 unsigned __int32
#define UINT64 unsigned __int64
#define UINT     unsigned int
#define BYTE    unsigned char


typedef  complex<float> float_complex;
typedef  complex<double> double_complex;
// WAS���������͸��ٲ������� 
#define BYTE_PER_TAR_WAS 16		// WAS-GMTI target byte size   // Modified by Luo 2020/03/04
#define WAS_MIN_VR  1.5
#define WAS_RD_LOC_WAY    false   // false -- ���ζ�λ�� true -- RD��λ��


// SAR-GMTIģʽ��������
#define BYTE_PER_TAR_MTI 10		// SAR-GMTI target byte size

#define GEOTIFF_VERSION true
#define ACCURATE_ML true					// SAR/GMTI���о�ȷ�Ķ��Ӵ���

#define GMTI_WIN_RG 12       // ���������۴���С
#define GMTI_WIN_AZ 96       // ��λ�����۴���С
#define GMTI_MA_CFAR true  // �Ƿ����MA-CFAR���м��

#define IMAGE_AUX_SIZE     512
#define TRACK_AUX_SIZE     1024
#define MAX_IMAGE_LEN      16777216
#define IMAGE_BUFFER_SIZE  48   // number of small picture we can store before track data came
#define TRACK_BUFFER_SIZE  16   // number of track data we can store before PRF match


// �������Ͷ���

#define ERROR_MODEL -1 //ģʽ����
#define ERROR_PATH  -2
#define ERROR_Search -3
#define ERROR_TIFF  -6
#define ERROR_ARRAY -5
#define ERROR_Value  8
#define MAX_RANGE_INTV 999999 //


#define SC_D (180.0)
#define FT_M 0.3048
#define C_LIGHT (3.0e8)

#define C          2.999999999e8 
#define PI         3.14159265358979
#define PI_D2      1.5707963267490 
#define MAX(a,b)  (a>=b ? (a) : (b))
#define MIN(a,b)  (a<=b ? (a) : (b))
#define INFO_NUM   8				// �����´�Ŀ���������
#define OUT_INFO_NUM 10		// ���������Ŀ�������������

#define LEFT_SIDE	 1
#define RIGHT_SIDE	-1

//sar-GMTIһ����Ŀ��ռ�õ��ֽ���
const int  GMTIAIMWIDTH=10;
const bool recordsPrint=true;

// for R-D locating 
#define DIM 3  // x(n)
#define EPSILON 1.0e-8
#define MAXITER 1000
#define C   2.999999999e8 
#define PI  3.14159265358979

//����GMTI��λ��ֵ���ȵ�����ֽ���

#define GYPHMLENMAX (512*8)
#define GYAIMLENMAX  (2048*6)		// ���ԭʼ�㼣����*6

const int  HD_LEN=4; 
const UINT32 HEADLEN=512;
const int    TIFHEIGHT=512;
const unsigned short sarGmtiAimPos=509;
const int DT_HEAD[]={0x01, 0xDC, 0xEF, 0x18};
const int DT6Suo_HEAD[]={0xAA, 0x55, 0xAA, 0x55};
const float BD_ARRAY[]={420,210,80,25,1360,700};
const float FS_ARRAY[]={500,250,125,31.25,1600,800};
const double RS_ARRAY[]={0.150,0.30,0.50,1.0,3.0,5.0,10.0,20.0};
const char MD_ARRAY[][16] ={"����","����","����","SAR-GMTI","����-GMTI"}; 
const double BEAM_WIDTH[]={2.067,2.77,3.22,6.4,3.1,2.067};


/************��¼��Ϣ֡λ��******************/

enum numTypeEnum{otherEnm,uInt8Enm,uInt16Enm,uInt32Enm,uInt64Enm,int16Enm,int32Enm,int64Enm,fl32Enm,db64Enm,Bcd1Enm,Bcd2Enm,Bcd3Enm,Bcd4Enm};

struct stParInFrame
{
		int startPosion;//ƫ����(�ֽ�)
		double unit;//(��λ)Ĭ��Ϊ1
		numTypeEnum numType; 
	    stParInFrame();
};

typedef struct gyAimInfoSt
{
	int aimsNum;
	double aimAmp1,aimAmp2;
	double aimInterPhase;	
	int aimDopNum;
	int aimRangeNum;
	int aimPostDopNum;

	double aimInterAngle;

	struct gyAimInfoSt * next;

    double tar_r0;
	double tar_ampl;
	double tar_azimuth;
	double href;

	float tar_snr;
	float tar_wid;
    
	int aim_frame_no;
	int aim_wave_no;

	double aim_direction;
	double aim_velocity;
	double aim_longitude;
	double aim_latitude;
	double aim_ts;

} gyAimInfoList;

typedef struct gyAimInfoHdSt
{
	gyAimInfoList  *gyAimInfoPt;
	int numAll;

}gyAimInfoHd;

typedef struct gyPathInfoSt
{
	int Id;
	int cnt;//���� �����˼�����
	double Longitude;
	double Latitude;
	double Height;
	double Velocity;
	double Direction;
	double IFNew;
	double Prop;
	double Snr;
	double HrrpWid;
	struct gyPathInfoSt *Next;

}gyPathList;

typedef struct gyPathHdSt
{
	double pathNum; //������Ŀ

	gyPathList *gyPathPt;

} gyPathHd;

typedef struct gyConInfoSt
{
	double dbLatitude,dbLongitude,vel,ID;

}gyConInfo;


typedef struct sys_par
{
	int frame_no;
	int wave_no;
	double B_Na;			//��Ҫ�ж�
	double R_near;		//���б����
	double prf;				//prf
	double az_vel;
	double e_v;
	double n_v;
	double u_v;
	double scan_ang;		// ����
	double elev_ang;		// ���� ������
	double yaw_ang;		// ����
	double lati;				//�ػ�γ��
	double logn;			//�ػ�����
	double fly_ang;			// �ػ������
	double Fs;
	double href;
	double lookside;  // ��--1���� -- -1;
	double fc;
	double Br;
	double D;
	double tar_height;
	double detect_th;
	long Nr;
	long burst_num;
	bool is_sea;
	double ts;

	float ch1_ph;
	float ch2_ph;

	float fdc_est;

} GYParsTOLuo;


/*******************************************/

class fmtConvCl
{
private:

	UINT8 *ar;
	union {
		UINT8 u[2];
		__int16 s;
	} Int16Un;
	union {
		UINT8 u[2];
		UINT16 s;
	} UInt16Un;

	union {
		UINT8 u[4];
		__int32 s;
	} Int32Un;

	union {
		UINT8 u[4];
		UINT32 s;
	} UInt32Un;

	union {
		UINT8 u[8];
		double s;
	} db64Un;

	union {
		UINT8 u[4];
		float s;
	} ft32Un;

	UINT16 byteToUint16(int start);	 
	 __int16 byteToInt16(int start);

	 //���
	 UINT16 b2U16BigEndian(int start);
	 __int16 b216BigEndian(int start);
	 //
	 UINT16 b2U16LittleEndian(int start);
	 __int16 b216LittleEndian(int start);
	 
	 __int32 byteToInt32(int start);	 

	 double byteToDouble(int start);
	 float byteToFloat(int start);

	 int   byteTo16Bcd(int start);
	 int   byteTo8Bcd(int start);

public:

	UINT32 byteToUint32(int start);

	UINT32 b2U32BigEndian(int start);

	fmtConvCl(UINT8 *ar);

	~fmtConvCl();

	double getResult(stParInFrame fmFrame);

	double getBigEndianResult(stParInFrame fmFrame);

	double get16LittleEndianResult(stParInFrame fmFrame);	
};

//��¼��ͷ����λ��
struct taskInfoPosition
{
		//������ţ����������ͣ��ɻ����ţ��ɻ��ţ����䷽ʽ��ͼ��ѹ����
		stParInFrame missionCodes,planeType,planeNum,transType,imgCompRate,powerOnCnt;

		void initial();
};

//��¼��ͷmuλ��
struct muInfoPosition
{       
	    //ɨ���кš��ꡢ�¡��ա�ʱ���֡��롢���� 
	    stParInFrame scanCode,date_year,date_month,date_day,time_hour,time_minutes,time_second,time_m_second,
		//���ȣ�γ�ȣ��ɻ��߶ȣ��ɻ�Ŀ����Ը߶ȣ��ɻ������ǣ��������ٶȣ������Ǽ��ٶ�
		plane_longitude,plane_latitude,plane_height,aim_height,plane_dive_angle,plane_dive_angle_V,
		//�ɻ�����ǣ�����ٶȣ�������ٶȣ��ɻ�����ǣ��ɻ�����ƫ��
		plane_dive_angle_a,plane_hor_angle,plane_hor_angle_v,plane_hor_angle_a,plane_direction_angle,plane_departure_angle,
		//�ɻ�ƫ���ǣ��ɻ�ƫ�����ٶȣ��ɻ�ƫ���Ǽ��ٶȣ����٣�����ٶȣ�ָʾ�ٶ�
		plane_de_flow_angle,plane_departure_v,plane_departure_a,plane_ground_v,plane_noair_v,plane_point_v,
		//�ɻ����ٶȣ��ɻ����ٶȣ��ɻ����ٶȣ�������٣�������٣��������
		plane_east_v,plane_north_v,plane_up_v,plane_east_a,plane_north_a,plane_up_a;
	    //rAngle��������Ԥ���Ƿ�ΧΪ(-179 ~179��)�������Ϊ�����Ҳ���Ϊ��  rAngle_pt--ʵ�ʸ���ָ��Ƕ�
	    stParInFrame rAngle,aAngle,aAngle_cal,rAngle_pt,rsAngle; 
	    
	    void initial();

};

//��¼��ͷͼ�����λ��
struct sarImageInfoPosition
{
	     //���ںţ������ţ� ͼ��������ͼ��������ͼ��λ�
	    stParInFrame loop_num,strip_num,img_cols,img_rows,img_deepth,
     	//ͼ�����ĵ㾭�ȣ�ͼ�����ĵ�γ�ȣ�ͼ�����ĵ�߶ȣ� ���ϽǾ��ȣ����Ͻ�γ�ȣ����½Ǿ���
		img_center_longitude,img_center_latitude,img_center_height,left_top_longitude,left_top_latitude,left_down_longitude,
	    //���½�γ�ȣ�         ���ϽǾ��ȣ�           ���Ͻ�γ�ȣ�   ���½Ǿ��ȣ�           ���½�γ��
		left_down_latitude,right_up_longitude,right_up_latitude,right_down_longitude,right_down_latitude,
		//����ģʽ�� ����  
		work_mode,look_Side,      
		//��λ��Ԫ�ߴ磬������Ԫ�ߴ磬�ؾ�ֱ��ʣ�б��ֱ��ʣ�ͼ����� ͼ��      ���б�࣬        ��Զб�࣬ ��������б�࣬       ����ؾ�
		a_point_size,r_point_size,ground_res,slope_res,img_width,img_length,slope_nearest,slope_far,slope_sen_center,ground_nearest,
		//��Զ�ؾ࣬ �������ĸ߶ȣ�����ָ��ǣ��ظ�Ƶ�ʣ�prf����������������Ƶ�ʣ�������������ҪĿ��ʶ���غɱ�ţ��غ�����
		ground_far,scene_center_height,spot_dir_angle,prf,prfCount,dopler_center,time_poweron,import_aim_bool,load_code,load_type,		
		//����ˮƽ��ȣ�
		beam_horz_width,beam_A_width,beam_R_width,beam_FW_Angle,
		//���� ����Ƶ�� ��������ӽ� �ֱ��� �������� �������ɻ����� �ɻ�γ�� �ɻ��߶� �ɻ�Ŀ��߶� �ɻ������ٶ� �ɻ������ٶ�
		band,fs,rsAngle,rAngle,rAngle_pt,sarRes,sampoints,multipleView,plane_longitude,plane_latitude,plane_height,aim_height,plane_east_v,plane_north_v;

		stParInFrame sampleStart,Tp,samples;

		void initial();

};

//��¼��ͷsarGMTI����λ��
struct sarGMTIAimInfoPosition
{   
	//Ŀ������ͼ��λ��������һ��Ŀ��Ĳ���
  	stParInFrame 	aimsNum, scAzPoints;
	
	void initial();
};

//��¼��ͷ����GMTI����λ��
struct GYParsInfoPosition
{
	//֡��š���λ�š���λ������ʼ�ǡ���λ������ֹ�ǡ�ɨ�貽����פ������������������������ģʽ���غɱ�š��غ�����	
	//�ֱ��� ��λ������� ������Ƶ ��������
	//ɨ�跶Χ ɨ���
	stParInFrame frameCodes,waveCodes,scanCenterAngle,scanScope,scanStep,pulseResident,powerOnTimes,workMode,loadCode,loadType,
		         resvAb,spotWid,prf,band,scanAngle,samples,azimuthCenterAngle;
	////�ɻ��߶� Ŀ��߶� ����б��  ����Ƶ��
	stParInFrame sampleStart,Tp;

	void initial();

};


struct GYAimInfoPosition
{
    //Ŀ����  is useless
  	stParInFrame aimsNum;	
	void initial();
};

//��¼��ͷ����GMTIĿ��λ��
struct NewGYAimInfoPosition
{
	 //Ŀ����
  	stParInFrame aimsAll,aimsNum,pulseResident,phaseMidStart,aimParsStart,detect_th;	

	void initial();
};

/************��¼������Ϣ******************/
//�����������
class taskInfo
{   
		public:
		//������ţ����������ͣ��ɻ����ţ��ɻ��ţ����䷽ʽ��ͼ��ѹ����
		double missionCodes,planeType,planeAllowNum,planeNum,transType,imgCompRate,powerOnCnt;
		
		taskInfo(UINT8 *ar);

		~taskInfo();

		int output(FILE *FL);

};

//IMU������
class muInfo
{       
        public:
	    //ɨ���кš��ꡢ�¡��ա�ʱ���֡��롢���� 
			double scanCode, date_year, date_month, date_day, time_hour, time_minutes, time_second, time_m_second,
				//���ȣ�γ�ȣ��߶ȣ��ɻ�Ŀ����Ը߶ȣ��ɻ������ǣ��������ٶȣ������Ǽ��ٶ�
				plane_longitude, plane_latitude, plane_height, plane_aim_height, plane_dive_angle, plane_dive_angle_V,
				//�ɻ�����ǣ�����ٶȣ�������ٶȣ��ɻ�����ǣ��ɻ�����ƫ��
				plane_dive_angle_a, plane_hor_angle, plane_hor_angle_v, plane_hor_angle_a, plane_direction_angle, plane_departure_angle,
				//�ɻ�ƫ���ǣ��ɻ�ƫ�����ٶȣ��ɻ�ƫ���Ǽ��ٶȣ����٣�����ٶȣ�ָʾ�ٶ�
				plane_de_flow_angle, plane_departure_v, plane_departure_a, plane_ground_v, plane_noair_v, plane_point_v,
				//�ɻ����ٶȣ��ɻ����ٶȣ��ɻ����ٶȣ�������٣�������٣�������� Ŀ��߶�
				plane_east_v,plane_north_v,plane_up_v,plane_east_a,plane_north_a,plane_up_a,aimH;
		
		double aAngle, azAngle_cal, rAngle, rsAngle,rAngle_pt;

		float cfar_th_dB;
		
		muInfo(UINT8 *ar);

		~muInfo();

		//int output(FILE *FL);
		int output(FILE * FL,int gdNum);

};

//SARͼ�����
class sarImageInfo
{
        public:
	    //���ں�,�����ţ�ͼ��������ͼ��������ͼ�����ͼ�񳤣�ͼ��λ�����ˮƽ��ȣ�������λ���ȡ������������ȣ�������λ�ǣ�ͼ�����ĵ㾭�ȣ�
	    double loop_num,strip_num,img_cols,img_rows,img_width,img_length,img_deepth,beam_horz_width,beam_A_width,beam_R_width,beam_FW_Angle,img_center_longitude,
		//ͼ�����ĵ�γ�ȣ�ͼ�����ĵ�߶ȣ��������ĸ߶ȣ����ϽǾ��ȣ����Ͻ�γ�ȣ����½Ǿ���
		img_center_latitude,img_center_height,scene_center_height,left_top_longitude,left_top_latitude,left_down_longitude,
		//���½�γ�ȣ����ϽǾ��ȣ����Ͻ�γ�ȣ����½Ǿ��ȣ����½�γ��
		left_down_latitude,right_up_longitude,right_up_latitude,right_down_longitude,right_down_latitude,
		//��λ��Ԫ�ߴ磬������Ԫ�ߴ磬�ؾ�� ���ʣ�б��ֱ��ʣ����б�࣬��Զб�࣬��������б�࣬����ؾ�
		a_point_size,r_point_size,ground_res,slope_res,slope_nearest,slope_far,slope_sen_center,ground_nearest,rangePoints,
		//��Զ�ؾ࣬����ָ��ǣ��ظ�Ƶ�ʣ�prf����������������Ƶ�ʣ�������������ҪĿ��ʶ�𣬹���ģʽ������ģʽ��ţ��غɱ�ţ��غ����� �ϳɿ׾�ʱ�� ������λ�� �����������
		ground_far,spot_dir_angle,prf,Nprf,prfCount,dopler_center,time_poweron,import_aim_bool,work_mode,work_mode_num,load_code,load_type,syApTime,a_angle_new,R_angle_new;
		//����Ƶ�ʣ�����
		double band,fs,rsAngle,azAngle, azAngle_cal,rAngle,rAngle_pt,sarRes,look_Side,multipleView,sampoints,plane_longitude,plane_latitude,plane_height,aim_height,plane_east_v,plane_north_v,plane_up_v,sampleStart,Tp,samples;
		
		char workModeStr[18];
		int sar_mode;

		double range_offset;
		int year,month,day;
		bool GMTI_OFFSET_ERR;
		double gps_info[10];

		double ml2_coef;

		sarImageInfo(UINT8 *ar);

		void sarImageLocating(double pre_lati,double pre_logn,double cur_lati, double cur_logn);

		int output(FILE *FL);
};

//SAR/GMTI������
class sarGMTIAimInfo
{
public:
	//�ɻ����ȡ��ɻ�γ�ȡ��ɻ��߶ȡ��ɻ�����ǡ����б�ࡢ
	double plane_longitude,plane_latitude,plane_height,plane_direction_angle,plane_ground_v,
		slope_nearest,fs,sampoints,elevationAngle,look_Side,prf,multipleView,yawAngle;
	double tar_height;
	//sarGMTI�㼣��ӦN��ͼ���,��N��ͼ�ķɻ���γ��
	double planeLonLast,planeLatLast;
	//Ŀ��λ����Ϣ
	stParInFrame *aimRangePointsFr,*aimAzimuthPointsFr,*aimVFr,*aimStrengthFr;
	//Ŀ�����
	UINT32 aimsNum;
	UINT32 scAzPoints;
	UINT32  * aimRangePoints;
	UINT16  * aimAzimuthPoints;
	
	//�����˱��У��
	int		*Tar_Azloc;
	char    *Tar_azrev;

	float   * aimV;
	UINT16  * aimStrength;
	float   * aimLongitude;
	float   * aimlatitude;

	UINT32  loopNum;

	//��ʾ��ʼ���Ƿ�����
    bool blInitial;

public:

	sarGMTIAimInfo(UINT8* ar);

	sarGMTIAimInfo(UINT8* ar,UINT8* arImage,double *gps_info, string tar_map_file);

	void img_location_cal(float *logn, float *lati,__int64 na, __int64 nr, int tar_num, double *gps_info, int *tar_az, char *tar_azrev, unsigned int *tar_ra);

	int Point_Trace2(UINT16 *Tar_Az,UINT32 *Tar_Ra,UINT16 *Tar_Amp, float *Tar_Vr,UINT Tar_Num,int win_ra,int win_az);
	
	int Point_Trace_Vr(UINT16 *Tar_Az,UINT32 *Tar_Ra,UINT16 *Tar_Amp, float *Tar_Vr,UINT Tar_Num,int win_ra,int win_az);

	~sarGMTIAimInfo();

	int output(FILE * FL);

};

//WAS-GMTIϵͳ����
class GYParsInfo
{// check by Yunhua-Luo @ 03/20
      
	public:
	//֡��š���λ�š���λ������ʼ�ǡ���λ������ֹ�ǡ�ɨ�貽����פ������������������������ģʽ���غɱ�š��غ�����
	//�ֱ��� ��λ������� ������Ƶ ���� ɨ��� ������ʼ  ����
	double frameCodes,waveCodes,waveNum,scanCenterAngle,scanScope,scanCycle,scanStep,pulseResident,powerOnTimes,workMode,loadCode,loadType,
		   resvAb,spotWid,prf,Nprf,band,fs,scanAngle,sampleStart,Tp,samples,azimuthAngle,azimuthCenterAngle,rg_beam_width,rg_ang,fly_ang;
	float Cfar_th_dB;
	//�������  ��Զ����
	double RNear,RFar;
	int output(FILE * FL);
	GYParsInfo(UINT8* ar);
	~GYParsInfo();
};

// WAS-GMTI�㼣��
class NewGYAimInfo
{
public:

	 unsigned char head[512];

	//Ŀ���������λ��פ����
	double aimsNum,aimsAll,detect_th,pulseResident;

	double phaseMiddleValue[GYPHMLENMAX],aimPars[GYAIMLENMAX];

	bool is_sea;

	float scanScope;

	gyAimInfoHd lstHeader;

	NewGYAimInfo(UINT8* ar);

	~NewGYAimInfo();
	
private:

	float Bit16ToDot(unsigned __int16 data);

	void InfoTxtOut(char *file, float *DotInfo,int Dot_num);

	template <class T>
	void median_phase_filter(T* phase,long len, int win_len);

	template <class T>
	void BubbleSort(T *pdata,__int64 N,__int64 *pIndex);

	template <class T>
	inline  double min_val(T *val,__int64 num,__int64 *loc);

	template <class T>
	void Write2Double(char* file_out,T* pdata,__int64 N);

	template <class T>
	inline void CircShift(T *data,long data_len,int mov_shift);

	int Point_Trace(long *Tar_Az,long *Tar_Ra,float *Tar_Amp, float *Tar_ph,long Tar_Num,int win_ra,int win_az);
	
	int Point_Trace_1D(long *Tar_Ra,float *Tar_Amp, float *Tar_SNR,long Tar_Num,int win_ra);
	
	int Point_SortByAmpl(long *Tar_Az, long *Tar_Ra, float *Tar_Amp, float *Tar_ph, long Tar_Num, int MaxTarNum);

	int WASmti_locating_v2020(double *tar_info, long tar_num, sys_par par); // added by Dr.luo 20200304
	
	int SettingPara(float &snr_th);

};


class AxisCal
{
public:

	void WGS84ToENV(double lati, double logn, double href, double lati_ref, double logn_ref, double* Pos);
	//void WGS84ToENV(double lati, double logn, double href, double* Pos);
	void ENVToWGS84(double lati, double logn, double* Pos, double* latlogn);
	double RangeCalByGPS(double lati0,double logn0, double lati1, double logn1);
	double CalDirection(double lat_s,double lon_s, double lat_e, double lon_e);

	void WGS84ToScene(double lati,double logn, double* pos);
	void SceneToWGS84(double Px, double Py, double lati_ref,double* latlogn);
};

// ����������
class NewGYLineInfo
{
public:

	int TrkNum;

	float plane_lati;

	float plane_logn;

	float CurScanAngle;

	gyPathHd linePathHeader;	// ��������ṹ

	unsigned char head[512];

	UINT8 *ar;

	int SettingTrackPara(NingjuPara & njPa, TrackPara & tkPa, KalManPara & KmPa);

	void InfoTxtOut(char *file, float *TrkInfo,int trk_num,int frame_no);
	
	// ******** functions **********
	NewGYLineInfo(UINT8 *arIn);

	~NewGYLineInfo();

	int MT_relate_XL_V2020(float *aimInfo, long tar_num, int scan_no,float scan_cycle);

};

// ************* image processing functions ******************
class imgProc
{
	
private:

	int norm_image(Mat & src);

	Mat mean_filter(Mat & src,int WinW);

	Mat var_filter(Mat & src, int WinW);

	Mat Lee(Mat& image_in,int WinW,float Looks);

	Mat Gamma(Mat& image_in,int WinW,float Looks);

	Mat Kuan(Mat& image_in,int WinW,float Looks);

	Mat Frost(Mat& image_in,int WinW);

	Mat Sigma(Mat& image_in,int WinW);

	template <class T>
	double max_val(T *val, long num, long *loc);

	template <class T>
	T MedianSort(T *data, int num);

	template <class T>
	void mov_aver_gai(T* pdata, __int64 N, __int64 win);

	template <class T>
	void RadativeCorrect(T *data, long na, long nr, float rn, float dr, float high);

	void smooth(unsigned char *img_data,__int64 img_na,__int64 img_nr);

	void gamma_correct(Mat &src,float high,float low, float gamma);

	void high_low(Mat &src ,float &low,float &high);

	void Gray_adjust(unsigned __int8 *pdata, __int64 len, __int64 hist_sum_th_ratio);

	void Gray_AlogQuan(unsigned __int8 *pdata, __int64 len);

	double sinc(double x);
	
	void cal_ampl_coef(float *ampl_coef,float dr,float r_near ,float href,float r_ang, float ra_beam_ang,long nr);
	
	void ampl_corr(unsigned char *img_data,float *ampl_coef,__int64 img_na,__int64 img_nr);
	
	void Local_Funcs(SAR_VS_params param);

	void Local_Funcs_Lookside(SAR_VS_params param, int lookside, double *gps_info, double *modeinfo);

	void Local_Funcs_Lookside(SAR_VS_params param, int lookside, double *gps_info); // added by luo 2016/2/26

	void geo_corr(unsigned char *img_data,float ra_res,float r_near,float href,__int64 img_na,__int64 img_nr);

	bool Mirror_Judge(unsigned char *data);

public:

	string rootPath;

	float az_ml2_coef;

	UINT8 *ar;

	imgProc(UINT8 *arIn,string strPath);

	imgProc();

	void SAR_VS_params_init(SAR_VS_params &params);

	int creatCoTif_New(UINT8 *dataAr,__int64 data_len); // rewrite by Zhang Lei

	void ImgScale(uchar *image_in, int &width, int &height, float width_fscale, float height_fscale);

	void ImgRotate(uchar *image_in, int width, int height, int angle,Mat& mat_out);

	//void ImgRotate(uchar *image_in, int &width, int &height, int angle);

	void ImageCorr_for6suoMap(unsigned char *pdata, __int64 row, __int64 col, int lookside);

	void allImageProMethod(char * strImage,char * strDest,imageProcParsST procPars);

	void FFT(float_complex *FFTbuf, long FFTLeg, int FFTMode);
	
};

class imgPreProc
{
	imgPreProc()
	{

	}

};

//����ģʽ���еĹ���
//�����ļ�����
class commonFuncs
{

 public:

	//�������_����(�£��գ���)_ʱ��(ʱ���֣���)_������_���ں�_������ѹ����ʶ�Լ��������_�ɻ����ŷɻ���_ѹ����_xxx��ʶ���غɱ�ʾ
	string  getName(UINT8 * ar);
	string  getNameByLoopNum(UINT8* ar,int stripNum);
	string  getNameLocalTime(UINT8* ar,int stripNum);

	//@20151126
	//��ȥ,������ʽ��Ҫ����
	//���������,���һ��������ʱ�����,�㲻д�Ĵ��붼������˼
	 string getRawPicName(UINT8 * ar);
	 string getDataName(UINT8 * ar);
	 string getDataName_luo(UINT8 * ar);
	 string getMarkPicName(UINT8 *ar);
	 //
	 string getRawPicName(UINT8 * ar,int stripNum);
	 string getCombPicName(UINT8 * ar,int stripNum);
	 string getDataName(UINT8 * ar,int stripNum);
	 string getMarkPicName(UINT8 *ar,int stripNum);

		//***pars:
		//dataAr:����Դ���飻
		//dthead:֡ͷ��־λ����;
		//arLen:����Դ����ĳ��ȣ�
		//headLen:֡ͷ��־λ����ĳ��ȣ�
	int  getHeadPosition(UINT8 *dataAr,int arLen);
	     //6Suo date
	int  getHead6SuoDatePosition(UINT8 *dataAr,int arLen);
	     //6suo frame length
	int  getHead6SuoDateFrameLen(UINT8 *dataAr);

	int  searchHeadPositionX64(FILE * fl);
	    //��FILE������
	int  searchHeadPosition(FILE * fl);
	    //��ȡsar�Ĺ���ģʽ
	int  getSarModel(UINT8 * ar);
	    //��ȡ�ֱ���ͼ�� Ԥ��ͨ������жϷ�λ����
	int  getSarRes(UINT8 * ar);

	    //�ж��Ƿ�sar/gmti��ͼ����Ϣ
	bool getIfSarGmtiTiff(UINT8 *ar);

	UINT64 getPRFNum(UINT8 * ar);
	     //��ȡ˫ͨ������GMTIĿ�����
	int getNewGYAimsNum(UINT8 * ar);

	int getMViewNum(UINT8 *ar);

	int getPowerOnTime(UINT8 * ar);

	int getLoopNum(UINT8 *ar);

	double getDopplerCenter(UINT8 *ar, double prf);

	void logRecords(char * chMessage,double cnt); //OK

	template <class T>
	void logRecords_mt(char* chMessage,T cnt);

	string  getSysTime(FILE *fl);

	int  getHeadPosition(UINT8 *dataAr,int arLen,int *rePos);

	__int64 getHeadPosition_i64(UINT8 *dataAr,__int64 arLen);

	int getWasMTIBagNo(UINT8 *dataAr);

	int getWasMTIBagNum(UINT8 *dataAr);

	int  getRangePoints(UINT8* ar);

	void ReadXL_EMapFile_mode_1(char* filename, char* outputdir);

	void ReadXL_EMapFile_mode_2(char* filename, char* outputdir);

	void ReadXL_EMapFile_mode_3(char* filename, char* outputdir);

	void ReadXL_EMapFile(char* filename, char* outputdir, int mode);

	void OutputRDLocatingInfo(FILE *fl, sarImageInfo *ps, sarImageInfo *pe);

};

class GMTIModel
{

public:

	double dr;
	double da;

	double gps_info[10];

	float fly_ang;

public:

	UINT8 *ar_last;
	UINT8* ar;
	UINT8* im_ar;

	string rootPath;

	//muticast
	WSADATA wsaData;  

	struct sockaddr_in servaddr;

	SOCKET sockfd,sockM;

public:

	bool overlay;
	long img_row;
	long img_col;

	sarGMTIAimInfo * gmtiAimPt;
	//sarGMTI�㼣��ӦN��ͼ���,��N��ͼ�ķɻ���γ��
	double lastPlaneLon,lastPlaneLat;

	double img_latis[4];
	double img_logns[4];
	double ml2_coef;

	GMTIModel(UINT8* arIn,string str);
	//~GMTIModel();

	int multicast_init();
  
	// add by zhang : add width and height information
	int outPutGMTIInfo(int loopNum, int nImgWidth, int nImgHeight, float az_ml2_coef);
	// int outPutGMTIInfo(int loopNum, int nImgWidth, int nImgHeight);

	int outPutGMTIData();
};

// WAS-GMTI process class 
class GY_GMTIModel
{

private:

	UINT8* ar;

	string rootPath;

	string dataName;

	int aimNum;

	int trkNum;

	NewGYLineInfo *gyNewLinePt;  // ����������

	NewGYAimInfo  *gyNewAim;		// Ŀ����

	//muticast
	WSADATA wsaData;  

	struct sockaddr_in servaddr;

	SOCKET sockfd,sockM;

public:

	int gy_Multicast_init();

	int outPut_NewGY_DatInfo();
	
	int outPut_NewGY_Line_DatInfo(unsigned char *ar);

	GY_GMTIModel(UINT8 *arIn,string str);
	
	~GY_GMTIModel();

	int Generate_WasMTIAim();
	
	int Generate_WasMTITrack();

};

class ImgMark
{

public:
		// add by zhang : up side down, mark with target-number and speed
	void SAR_GmtiTar_Mark(char *szMarkFile,char *szGrayFile,unsigned char *imgData,long *tar_az,long *tar_ra, char *tar_flag, float *tarVr,__int64 nImgWidth,__int64 nImgHeight,int nTarNum, int lookside, double *img_latis, double *img_logns);
	
	template <class T>
	double min_val(T *val,__int64 num,__int64 *loc);

	template <class T>
	double mean_val(T *val,long num);
	
	template <class T>
	double std_val(T *val,long num);

	template <class T>
	double max_val(T *val,long num,long *loc);

	template <class T>
	double  sinc(T x);

	int SAR_FOPEN(FILE **fid, char *fn, char *mode);

	int SAR_FREAD(FILE **fid, void *buffer, size_t size);

	int SAR_FWRITE(FILE **fid, void *buffer, size_t size);

	int SAR_FCLOSE(FILE **fid);

	int SAR_FSEEK(FILE **fid, __int64 offset, int origin);

	int Form_Gray8_Tif(unsigned char *img_data,char *img_file,long high,long wid);

	int Form_RGB8_Tif(unsigned char *img_data,char *img_file,long high,long wid);

	template <class T1, class T2>
	void Data2Quan(T1 *pdata, __int64 na, __int64 nr, T2 *pout, char bit, __int64 hist_sum_th);

};

// ***************** added by luo *************
struct rd_par_input{
	// input r-d parameters ok

	double lati;
	double logn;

	double vn;
	double ve;
	double vu;

	double fdc;
	
	double r0;
	double href;
	double h_gnd;

	double lambda;
	double lookside;

};

struct rd_par{
	// temp r-d parameters ok

	double Px;
	double Py;
	double Pz;

	double Vx;
	double Vy;
	double Vz;
	
	double rs;
	double lambda;
	double fdc;
	double h_gnd;

};

class Tarlocate{

private:

	void ff(double x[DIM], double f[DIM], rd_par par);
	int gcpelim( int process, double A[DIM][DIM], double xx[DIM]);
	int secant2( double x[DIM], rd_par par);
	void ENVtoECEF(double lati, double logn, double VE, double VN, double VU, double *Vout);
	void WGS84toECEF(double lati, double logn, double href, double *Pos);
	void ECEFtoWGS84(double Px, double Py, double Pz, double lati_ref, double *latlogn);
	double CalDirection(double lat_s,double lon_s, double lat_e, double lon_e);

public:

	void ArbitaryPosCal(double *pos_info, long *grid_info, long px, long py, double *latlogn);
	void Geo_locating(rd_par_input par_in, double *latlogn);
	void RD_locating(rd_par_input par_in, double *latlogn_rd, double *latlogn_geo);

};

class GeoTiff{

public:
	
	int GeotiffWriteFromFile(const char *infname,const char *outfname,double *adfGeoTransform,char *WKT);
	
	int GeotiffWriteFromByteData(BYTE *pBuffer,long imgH, long imgW, const char *outfname,double *adfGeoTransform,char *WKT);

	int GeotiffWriteFrom16BitData(unsigned short int *pBuffer,long imgH, long imgW, const char *outfname,double *adfGeoTransform,char *WKT);

public:	

	int Tiff2Geotiff(double *point_lati, double *point_logn, long imgW, long imgH, const char *InImagefile,const char *OutImagfile);

	int WriteByte2Geotiff(double *point_lati, double *point_logn, long imgW, long imgH, BYTE *data,const char *OutImagfile);

	int Write16Bit2Geotiff(double *point_lati, double *point_logn, long imgW, long imgH, unsigned short int *data,const char *OutImagfile);

	int Read2Geotiff(const char *infname, BYTE *pBuffer, long *info, double *adfGeoTransform);
};



// *************** declaration global functions ****************

template <class T>
T round(T val);

template <class T>
void swap_s(T &a, T &b,__int64 &index1,__int64 &index2);

template <class T>
int Partition(T* A,__int64 p,__int64 r,__int64 *index);

template <class T>
void QuickSort(T* R,__int64 low,__int64 high,__int64* index);

template <class T>
void median_phase_filter_g(T* phase,long len, int win_len);

template <class T>
void BubbleSort_g(T *pdata,__int64 N,__int64 *pIndex);


// **************** Implemetation of functions *****************

template <class T>
T round(T val)
{
	if(val >= 0) 
		return (int(val+0.5));
	if(val < 0) 
		return (-int(-val+0.5));
}

//quickly sort
template <class T>
void swap_s(T &a, T &b,__int64 &index1,__int64 &index2)
{
	T temp;
	temp = a;
	a = b;
	b = temp;

	__int64 temp_index;
	temp_index = index1;
	index1 = index2;
	index2 = temp_index;
}

template <class T>
int Partition(T* A,__int64 p,__int64 r,__int64 *index)
{
	float x = abs(A[r]);
	__int64 i = p;
	__int64 j = p;
	for(;j<r;++j)
	if(abs(A[j])>=x)
	{
		if(i!=j) swap_s(A[i],A[j],index[i],index[j]);
		i++;
	}
	swap_s(A[i],A[j],index[i],index[j]);
	return i;
}

template <class T>
void QuickSort(T* R,__int64 low,__int64 high,__int64* index)
{
	__int64 pivotpos;
	if(low<high)
	{
		pivotpos = Partition(R,low,high,index);
		QuickSort(R,low,pivotpos-1,index);
		QuickSort(R,pivotpos+1,high,index);
	}
}

template <class T>
void median_phase_filter_g(T* phase,long len, int win_len)
{// median phase filter for real-time WAS-GMTI mode
		 // dev by Yunhua-Luo @ 2016/3/30

			if(win_len <= 3)  return ;
			int M = win_len;
			T*    temp  = new T[M];
			T*    py    = new T[len];
			__int64* index = new __int64[M];
	
			// median filter 
			memcpy(py,phase,sizeof(T)*len);

			int L;
			for(int i = 0; i < M/2; i++)
			{
				L = M/2+3-i;
				memcpy(temp,phase+i,sizeof(T)*L);
				BubbleSort(temp,L,index);
				py[i] = temp[index[int(L/2)]];
			}

			for(int i = M/2; i < len-M/2; i++)
			{
				memcpy(temp,phase+i-M/2,sizeof(T)*M);
				BubbleSort(temp,M,index);
				py[i] = temp[index[int(M/2)]];
			}


			for(int i = len-M/2; i < len-3; i++)
			{
				L = len-i-1;
				memcpy(temp,phase+i,sizeof(T)*L);
				BubbleSort(temp,L,index);
				py[i] = temp[index[int(L/2)]];
			}

			memcpy(phase,py,sizeof(T)*len);

			delete [] py;
			delete [] index;
			delete [] temp;

		}

template <class T>
void BubbleSort_g(T *pdata,__int64 N,__int64 *pIndex)
{// OK

	__int64 kk,jj,ind;
	T temp;

	for(kk=0; kk<N; kk++)
	{
		pIndex[kk] = kk;
	}

	for(kk=0; kk<N; kk++)
	{
		for(jj=kk+1; jj<N; jj++)
		{
			if(pdata[kk]<pdata[jj])
			{
				temp = pdata[kk];	
				pdata[kk] = pdata[jj];
				pdata[jj] = temp;

				ind = pIndex[kk];
				pIndex[kk] = pIndex[jj];
				pIndex[jj] = ind;
			}
		}
	}
}

