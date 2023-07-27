#ifndef _MULTARTRK_H_
#define _MULTARTRK_H_
#include "stdafx.h"
#include<list>
#include <vector>
#include <queue>
#include <map>
//#include "Eigen/Dense"

using namespace std;
//using namespace Eigen;

#define MIN_TARCKNO (101)
#define MAX_TRACKNO	(10000)

#define MAX_PRDCIT_SPEEDX (400)  //lei
#define MAX_PRDCIT_SPEEDY (400)
#define MAX_PRDCIT_SPEEDZ (400)

//-----------------------------------常量定义--------------------------------------
#define PI				(3.1415926535897932384626433832795)  //圆周率

//-----------------------------------数据结构定义----------------------------------
typedef double FudianType; //20190126从float改为double

//卡尔曼滤波参数20190802
struct KalManPara{
	float Mr; //测量噪声矩阵距离
	float Ma; //测量噪声矩阵方位
	float Me; //测量噪声矩阵俯仰
	float Qs; //过程噪声功率谱密度
	float deltaA;//关联时预测点和测量点的方位角差阈值 20190806
	KalManPara():Mr(100.0),Ma(100.0),Me(100.0),Qs(1.0),deltaA(20.0){} //20190807修改默认值
};

#ifndef _RAWPOINT
#define _RAWPOINT

typedef struct _Rawpoint
{//原始点迹

	double R;//距离
	double A;//方位角
	double E;//俯仰角

	double In;//强度
	double Vr;//径向速度
	double ts;//时间戳

	double x;//大地坐标系x
	double y;//大地坐标系y
	double z;//大地坐标系z

	double lati;
	double logn;
	double alt;

	int   wave_cnt;//波位数
	int   frame_no;

}RawPoint;

#endif;

typedef struct _Trackpoint
{//航迹点

	//输出时用
	unsigned char type;//目标类型
	int track_no;//航迹编号
	int trust_miss_cnt;//确认航迹未能连续关联次数
	FudianType R;//距离
	FudianType A;//方位角
	FudianType E;//俯仰角
	FudianType longti;//经度
	FudianType lati;//纬度
	FudianType alt;  //海拔
	FudianType ts;//时间戳

	float SNR; 
	float HrrpWid;
	FudianType In;//目标强度
	FudianType Vr;//径向速度
	FudianType Heading;//航向

	//航迹关联时用
	FudianType x;//x坐标
	FudianType y;//y坐标
	FudianType z;//z坐标
	FudianType Vx;//x轴速度分量
	FudianType Vy;//y轴速度分量
	FudianType Vz;//Z轴速度分量

	FudianType P[6][6];//保存3维卡尔曼滤波器状态协方差矩阵

	int trust_relate_cnt;//确认航迹连续关联次数 

	int	wave_cnt;//波位数
	int frame_no;

}TrackPoint;

#define MAX_TEMP_TRACK_CNT		(10) //缓存的最大临时航迹点个数

typedef struct _TrackPointsTemp
{//临时航迹点

	//航迹管理m/n逻辑法的计数值
	unsigned char m_cnt;
	unsigned char n_cnt;
	unsigned char miss_cnt;//连续预测错误计数

	//当前位置计数
	unsigned char cur_pos_cnt;
	TrackPoint data[MAX_TEMP_TRACK_CNT];
}TrackPointsTemp;

typedef struct _TrackPara
{
	//航迹头管理用
	FudianType Vmax;//最大速度
	FudianType Vmin;//最小速度
	FudianType T;	//扫面周期

	//临时航迹管理和确认航迹管理用
	FudianType EchoGateSize;//波门大小,单位m

	//临时航迹管理滑窗m/n参数
	unsigned char m_tmp;
	unsigned char n_tmp;
	unsigned char miss_cnt_tmp;//连续预测错误计数
	unsigned char miss_cnt_trust;

}TrackPara;


typedef struct _NingjuPara
{//定义了点击凝聚用的矩形滑窗
	int Rwidth;//距离门单元数
	int Awidth;//方位单元数
	int Ewidth;//俯仰单元数
	FudianType Xstep;//距离步进
	FudianType Ystep;//方位步进
	FudianType Zstep;//俯仰步进
}NingjuPara;


typedef struct _PolarCoordPoint
{//极坐标点

	FudianType R;//距离
	FudianType A;//方位
	FudianType E;//俯仰
}PolarCoordPoint;


typedef struct _XYZCoordPoint
{//笛卡尔坐标点

	FudianType x;//x坐标
	FudianType y;//y坐标
	FudianType z;//z坐标
}XYZCoordPoint;

typedef struct _LongLatiAlt
{//经纬度和海拔

	FudianType longti;//经度
	FudianType lati;//纬度
	FudianType alt;//海拔
}LongLatiAlt;


typedef struct{
	bool IsInGate;//是否落在波门内
	FudianType dis;//点迹距离预测点的距离/或者关联概率
	double deltaA; //预测点和测量点之间的角度差 20190806
}DisInfo;

typedef struct
{
	int				plane_no;//阵面编号
	LongLatiAlt		local_lonLa;//当地经纬度 20190126修改，还需要高度
	unsigned char	imu_stat;//IMU安装方式，决定了不同的坐标系
	FudianType		err_angle;//安装误差

	char			imu_src;//imu来源选择 0 使用辅助数据imu 1使用显控imu
}ImuPara;//惯导参数

namespace RadarMtt
{
	class MultiTragetsTrack
	{
	public:
		MultiTragetsTrack():
		  m_tk_no (MIN_TARCKNO)
		  {
		  }

		MultiTragetsTrack(const int &track_no);

		MultiTragetsTrack(const NingjuPara &njPa,const TrackPara &tkPa);//拷贝构造函数

		__declspec(dllexport) void Input(vector<RawPoint> *rawpoints, const NingjuPara &njPa, const TrackPara &tkPa, const KalManPara &kmPa);	//输入

		__declspec(dllexport) vector<TrackPoint> *Solution();//整体解决方案(航迹关联)，输出航迹点


private:

		KalManPara m_km_para; //卡尔曼滤波参数
		NingjuPara m_nj_para;//凝聚参数
		TrackPara m_tk_para;//航迹关联参数
		ImuPara m_imu_para;//惯导参数
		vector<RawPoint> m_raw_pts;	//原始点迹 
		list<TrackPoint> m_njed_pts; //凝聚后的点迹
		list<TrackPoint> m_tk_head;//航迹头列表
		list<TrackPointsTemp> m_tk_tmp;//临时航迹列表
		list<queue<TrackPoint>> m_tk_trust;//确认航迹列表
		int m_tk_no;//航迹编号

	public:

		float plane_lati;
		float plane_logn;
		float tar_ref_alt;
	
		//输出的航迹点列表
		vector<TrackPoint> m_tk_outs;

private:

		void GetTrackVelHeading(TrackPoint &pcur, TrackPoint &ppre, double Cycle_T);
	
		//bool cmpIn(const RawPoint &p1, const RawPoint &p2);
		/*****************************************1.点迹凝聚***********************************/
		void DianjiNingjuGeneralFast();					//20190806 先对点迹列表进行幅度排序之后再处理，这样可以避免每次查找最大值
		//原始点迹转为航迹点
		void RawPoint2TrackPoint(const RawPoint &rp,TrackPoint &tp);
		/*****************************************2.坐标系变换***********************************/
private:
		//2.1球坐标转为笛卡尔坐标
		static void Polar2XYZ(const PolarCoordPoint &pp,XYZCoordPoint *xyzp);
		//2.2笛卡尔坐标转为球坐标
		static void XYZ2Polar(const XYZCoordPoint &xyzp,PolarCoordPoint *pp);
		//2.5地球坐标系转为经纬度(大地坐标系)
		static void ECEF2WGS84(const XYZCoordPoint &earth,LongLatiAlt *longlaalt);
		//大地坐标系转到空间直角坐标系
		static  void BLHtoXYZ(const double &B, const double &H, const double &L, double *X, double *Y, double *Z);
		
		static void ECEFToENV(double lati, double logn, double Vx, double Vy, double Vz, double *Vout);
		
		//空间直角坐标系转到大地坐标系
		static  void _XYZtoBLH(const double &X,const double &Y,const double &Z,double *B,double *H,double *L);
		//解算经纬度
		void CalcLongLati(const XYZCoordPoint &radar,LongLatiAlt *tarLonLatAlt);
		//填充航迹点经纬度部分
		void FillLongLati(TrackPoint &tp);
		void FillXYZByRAE(TrackPoint &tp);
		void PredcitNextPointLine(const TrackPoint old_point,const FudianType period,TrackPoint *next_point);
		//航迹头管理
		void TrackHeaderManage();
		//临时航迹管理
		void TrackTempManage();
		//确认航迹管理
		void TrackTrustManage();
		//航迹头转为临时航迹
		void TrackHeader2TrackTemp(const TrackPoint &ningjued_pt,const TrackPoint &track_head);
		//临时航迹转为确认航迹
		void TrackTemp2TrackTrust(TrackPoint &flt_point,const list<TrackPointsTemp>::iterator it_tmp);
		//临时航迹转为确认航迹（最后一个点时预测外推点）20190116
		void TrackTemp2TrackTrust(const list<TrackPointsTemp>::iterator it_tmp);
		//将点迹列表加入到航迹头列表中
		void AddRawPointToTrackHeader();
		//卡尔曼滤波器
		void KalmanFilter(TrackPoint measure_val,TrackPoint flt_val_old,FudianType T,TrackPoint &flt_val);
		//通过航迹点的XYZ坐标填充其RAE坐标，供航迹输出使用
		void FillRAEbyXYZ(TrackPoint &tp_flt);
		//寻找凝聚点迭代器位置
		list<TrackPoint>::iterator SeekNingjuIter(int no);
		//寻找临时航迹迭代器位置
		list<TrackPointsTemp>::iterator SeekTracktmpIter(const int &no,const vector<bool> &isMatch);
		//寻找确认航迹迭代器位置
		list<queue<TrackPoint>>::iterator SeekTracktrustIter(const int &no,const vector<bool> &isMatch);
		//临时航迹滤波外推
		void  TrackTempFltPre(list<TrackPointsTemp>::iterator it_tmp,const list<TrackPoint>::iterator it_nj);
		//临时航迹外推（不滤波，直接使用测量值)
		void  TrackTempNotFltPre(list<TrackPointsTemp>::iterator it_tmp,list<TrackPoint>::iterator it_nj);
		//确认航迹滤波外推
		void  TrackTrustFltPre(list<queue<TrackPoint>>::iterator it_trust,list<TrackPoint>::iterator it_nj);
		//计算点迹与临时航迹预测点之间的距离矩阵
		void CalcTempDisMat(vector<vector<DisInfo>> &dis_tab,vector<TrackPoint> &predict_list);
		//计算点迹与确认航迹预测点之间的距离矩阵
		void CalcTrustDisMat(vector<vector<DisInfo>> &dis_tab,vector<TrackPoint> &predict_list);
		//计算点迹与临时航迹预测点之间的概率矩阵 20190715
		void CalcTempProbMat(vector<vector<DisInfo>> &dis_tab,vector<TrackPoint> &predict_list);
		//根据距离矩阵处理临时航迹
		void DealTrackTemp(vector<vector<DisInfo>> dis_tab,vector<TrackPoint> predict_list);
		//根据距离矩阵处理确认航迹
		void DealTrackTrust(vector<vector<DisInfo>> dis_tab,vector<TrackPoint> predict_list);
		//判断距离矩阵中是否存在落在预测波门内的点
		bool HasValidPoint(const vector<vector<DisInfo>> &dis_tab);
		//设置匹配航迹列表的一项，设置为true 20190119
		void SetMatchList(int cnt,vector<bool> &MatchList);
		//封装Eigen库矩阵转置、求逆等操作

		static void matd_inv(double* src,double *dst,int m);//矩阵求逆
		static void matd_mpy(double* a,double *b,double *c,int row,int mid,int col);//矩阵乘法
		static void matd_add(double* a,double *b,double *c,int row,int col);//矩阵加法
		static void matd_sub(double* a,double *b,double *c,int row,int col);//矩阵减法
		//初始化航迹点
		void InitTrackPoint(TrackPoint &tp);


	}; // end of class
}// end of namespace




#endif

