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

//-----------------------------------��������--------------------------------------
#define PI				(3.1415926535897932384626433832795)  //Բ����

//-----------------------------------���ݽṹ����----------------------------------
typedef double FudianType; //20190126��float��Ϊdouble

//�������˲�����20190802
struct KalManPara{
	float Mr; //���������������
	float Ma; //������������λ
	float Me; //��������������
	float Qs; //���������������ܶ�
	float deltaA;//����ʱԤ���Ͳ�����ķ�λ�ǲ���ֵ 20190806
	KalManPara():Mr(100.0),Ma(100.0),Me(100.0),Qs(1.0),deltaA(20.0){} //20190807�޸�Ĭ��ֵ
};

#ifndef _RAWPOINT
#define _RAWPOINT

typedef struct _Rawpoint
{//ԭʼ�㼣

	double R;//����
	double A;//��λ��
	double E;//������

	double In;//ǿ��
	double Vr;//�����ٶ�
	double ts;//ʱ���

	double x;//�������ϵx
	double y;//�������ϵy
	double z;//�������ϵz

	double lati;
	double logn;
	double alt;

	int   wave_cnt;//��λ��
	int   frame_no;

}RawPoint;

#endif;

typedef struct _Trackpoint
{//������

	//���ʱ��
	unsigned char type;//Ŀ������
	int track_no;//�������
	int trust_miss_cnt;//ȷ�Ϻ���δ��������������
	FudianType R;//����
	FudianType A;//��λ��
	FudianType E;//������
	FudianType longti;//����
	FudianType lati;//γ��
	FudianType alt;  //����
	FudianType ts;//ʱ���

	float SNR; 
	float HrrpWid;
	FudianType In;//Ŀ��ǿ��
	FudianType Vr;//�����ٶ�
	FudianType Heading;//����

	//��������ʱ��
	FudianType x;//x����
	FudianType y;//y����
	FudianType z;//z����
	FudianType Vx;//x���ٶȷ���
	FudianType Vy;//y���ٶȷ���
	FudianType Vz;//Z���ٶȷ���

	FudianType P[6][6];//����3ά�������˲���״̬Э�������

	int trust_relate_cnt;//ȷ�Ϻ��������������� 

	int	wave_cnt;//��λ��
	int frame_no;

}TrackPoint;

#define MAX_TEMP_TRACK_CNT		(10) //����������ʱ���������

typedef struct _TrackPointsTemp
{//��ʱ������

	//��������m/n�߼����ļ���ֵ
	unsigned char m_cnt;
	unsigned char n_cnt;
	unsigned char miss_cnt;//����Ԥ��������

	//��ǰλ�ü���
	unsigned char cur_pos_cnt;
	TrackPoint data[MAX_TEMP_TRACK_CNT];
}TrackPointsTemp;

typedef struct _TrackPara
{
	//����ͷ������
	FudianType Vmax;//����ٶ�
	FudianType Vmin;//��С�ٶ�
	FudianType T;	//ɨ������

	//��ʱ���������ȷ�Ϻ���������
	FudianType EchoGateSize;//���Ŵ�С,��λm

	//��ʱ����������m/n����
	unsigned char m_tmp;
	unsigned char n_tmp;
	unsigned char miss_cnt_tmp;//����Ԥ��������
	unsigned char miss_cnt_trust;

}TrackPara;


typedef struct _NingjuPara
{//�����˵�������õľ��λ���
	int Rwidth;//�����ŵ�Ԫ��
	int Awidth;//��λ��Ԫ��
	int Ewidth;//������Ԫ��
	FudianType Xstep;//���벽��
	FudianType Ystep;//��λ����
	FudianType Zstep;//��������
}NingjuPara;


typedef struct _PolarCoordPoint
{//�������

	FudianType R;//����
	FudianType A;//��λ
	FudianType E;//����
}PolarCoordPoint;


typedef struct _XYZCoordPoint
{//�ѿ��������

	FudianType x;//x����
	FudianType y;//y����
	FudianType z;//z����
}XYZCoordPoint;

typedef struct _LongLatiAlt
{//��γ�Ⱥͺ���

	FudianType longti;//����
	FudianType lati;//γ��
	FudianType alt;//����
}LongLatiAlt;


typedef struct{
	bool IsInGate;//�Ƿ����ڲ�����
	FudianType dis;//�㼣����Ԥ���ľ���/���߹�������
	double deltaA; //Ԥ���Ͳ�����֮��ĽǶȲ� 20190806
}DisInfo;

typedef struct
{
	int				plane_no;//������
	LongLatiAlt		local_lonLa;//���ؾ�γ�� 20190126�޸ģ�����Ҫ�߶�
	unsigned char	imu_stat;//IMU��װ��ʽ�������˲�ͬ������ϵ
	FudianType		err_angle;//��װ���

	char			imu_src;//imu��Դѡ�� 0 ʹ�ø�������imu 1ʹ���Կ�imu
}ImuPara;//�ߵ�����

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

		MultiTragetsTrack(const NingjuPara &njPa,const TrackPara &tkPa);//�������캯��

		__declspec(dllexport) void Input(vector<RawPoint> *rawpoints, const NingjuPara &njPa, const TrackPara &tkPa, const KalManPara &kmPa);	//����

		__declspec(dllexport) vector<TrackPoint> *Solution();//����������(��������)�����������


private:

		KalManPara m_km_para; //�������˲�����
		NingjuPara m_nj_para;//���۲���
		TrackPara m_tk_para;//������������
		ImuPara m_imu_para;//�ߵ�����
		vector<RawPoint> m_raw_pts;	//ԭʼ�㼣 
		list<TrackPoint> m_njed_pts; //���ۺ�ĵ㼣
		list<TrackPoint> m_tk_head;//����ͷ�б�
		list<TrackPointsTemp> m_tk_tmp;//��ʱ�����б�
		list<queue<TrackPoint>> m_tk_trust;//ȷ�Ϻ����б�
		int m_tk_no;//�������

	public:

		float plane_lati;
		float plane_logn;
		float tar_ref_alt;
	
		//����ĺ������б�
		vector<TrackPoint> m_tk_outs;

private:

		void GetTrackVelHeading(TrackPoint &pcur, TrackPoint &ppre, double Cycle_T);
	
		//bool cmpIn(const RawPoint &p1, const RawPoint &p2);
		/*****************************************1.�㼣����***********************************/
		void DianjiNingjuGeneralFast();					//20190806 �ȶԵ㼣�б���з�������֮���ٴ����������Ա���ÿ�β������ֵ
		//ԭʼ�㼣תΪ������
		void RawPoint2TrackPoint(const RawPoint &rp,TrackPoint &tp);
		/*****************************************2.����ϵ�任***********************************/
private:
		//2.1������תΪ�ѿ�������
		static void Polar2XYZ(const PolarCoordPoint &pp,XYZCoordPoint *xyzp);
		//2.2�ѿ�������תΪ������
		static void XYZ2Polar(const XYZCoordPoint &xyzp,PolarCoordPoint *pp);
		//2.5��������ϵתΪ��γ��(�������ϵ)
		static void ECEF2WGS84(const XYZCoordPoint &earth,LongLatiAlt *longlaalt);
		//�������ϵת���ռ�ֱ������ϵ
		static  void BLHtoXYZ(const double &B, const double &H, const double &L, double *X, double *Y, double *Z);
		
		static void ECEFToENV(double lati, double logn, double Vx, double Vy, double Vz, double *Vout);
		
		//�ռ�ֱ������ϵת���������ϵ
		static  void _XYZtoBLH(const double &X,const double &Y,const double &Z,double *B,double *H,double *L);
		//���㾭γ��
		void CalcLongLati(const XYZCoordPoint &radar,LongLatiAlt *tarLonLatAlt);
		//��亽���㾭γ�Ȳ���
		void FillLongLati(TrackPoint &tp);
		void FillXYZByRAE(TrackPoint &tp);
		void PredcitNextPointLine(const TrackPoint old_point,const FudianType period,TrackPoint *next_point);
		//����ͷ����
		void TrackHeaderManage();
		//��ʱ��������
		void TrackTempManage();
		//ȷ�Ϻ�������
		void TrackTrustManage();
		//����ͷתΪ��ʱ����
		void TrackHeader2TrackTemp(const TrackPoint &ningjued_pt,const TrackPoint &track_head);
		//��ʱ����תΪȷ�Ϻ���
		void TrackTemp2TrackTrust(TrackPoint &flt_point,const list<TrackPointsTemp>::iterator it_tmp);
		//��ʱ����תΪȷ�Ϻ��������һ����ʱԤ�����Ƶ㣩20190116
		void TrackTemp2TrackTrust(const list<TrackPointsTemp>::iterator it_tmp);
		//���㼣�б���뵽����ͷ�б���
		void AddRawPointToTrackHeader();
		//�������˲���
		void KalmanFilter(TrackPoint measure_val,TrackPoint flt_val_old,FudianType T,TrackPoint &flt_val);
		//ͨ���������XYZ���������RAE���꣬���������ʹ��
		void FillRAEbyXYZ(TrackPoint &tp_flt);
		//Ѱ�����۵������λ��
		list<TrackPoint>::iterator SeekNingjuIter(int no);
		//Ѱ����ʱ����������λ��
		list<TrackPointsTemp>::iterator SeekTracktmpIter(const int &no,const vector<bool> &isMatch);
		//Ѱ��ȷ�Ϻ���������λ��
		list<queue<TrackPoint>>::iterator SeekTracktrustIter(const int &no,const vector<bool> &isMatch);
		//��ʱ�����˲�����
		void  TrackTempFltPre(list<TrackPointsTemp>::iterator it_tmp,const list<TrackPoint>::iterator it_nj);
		//��ʱ�������ƣ����˲���ֱ��ʹ�ò���ֵ)
		void  TrackTempNotFltPre(list<TrackPointsTemp>::iterator it_tmp,list<TrackPoint>::iterator it_nj);
		//ȷ�Ϻ����˲�����
		void  TrackTrustFltPre(list<queue<TrackPoint>>::iterator it_trust,list<TrackPoint>::iterator it_nj);
		//����㼣����ʱ����Ԥ���֮��ľ������
		void CalcTempDisMat(vector<vector<DisInfo>> &dis_tab,vector<TrackPoint> &predict_list);
		//����㼣��ȷ�Ϻ���Ԥ���֮��ľ������
		void CalcTrustDisMat(vector<vector<DisInfo>> &dis_tab,vector<TrackPoint> &predict_list);
		//����㼣����ʱ����Ԥ���֮��ĸ��ʾ��� 20190715
		void CalcTempProbMat(vector<vector<DisInfo>> &dis_tab,vector<TrackPoint> &predict_list);
		//���ݾ����������ʱ����
		void DealTrackTemp(vector<vector<DisInfo>> dis_tab,vector<TrackPoint> predict_list);
		//���ݾ��������ȷ�Ϻ���
		void DealTrackTrust(vector<vector<DisInfo>> dis_tab,vector<TrackPoint> predict_list);
		//�жϾ���������Ƿ��������Ԥ�Ⲩ���ڵĵ�
		bool HasValidPoint(const vector<vector<DisInfo>> &dis_tab);
		//����ƥ�亽���б��һ�����Ϊtrue 20190119
		void SetMatchList(int cnt,vector<bool> &MatchList);
		//��װEigen�����ת�á�����Ȳ���

		static void matd_inv(double* src,double *dst,int m);//��������
		static void matd_mpy(double* a,double *b,double *c,int row,int mid,int col);//����˷�
		static void matd_add(double* a,double *b,double *c,int row,int col);//����ӷ�
		static void matd_sub(double* a,double *b,double *c,int row,int col);//�������
		//��ʼ��������
		void InitTrackPoint(TrackPoint &tp);


	}; // end of class
}// end of namespace




#endif

