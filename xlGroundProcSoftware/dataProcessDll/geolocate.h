#ifndef __GEOLOCATE_H__
#define __GEOLOCATE_H__

// check ok 20200712

#pragma once

#include <vector>

// DEM file location : RES\\CNDEM.dat

using namespace std;
using std::vector;

#define RE          6378137.0           /* earth semimajor axis (WGS84) (m) */
#define FE          (1.0/298.257223563) /* earth flattening (WGS84) */
#define DIM 3  // x(n)
#define EPSILON 1.0e-8
#define MAXITER 1000

#define DEM_CenterNum 153
#define DEM_BlkNum 180
#define DEM_BlkSize 6000


typedef struct _CfarPoint
{// CFAR检测后的输出点迹信息

	long  az_pos;
	long  nr_pos;
	long  time;
	float In; //  
	float SNR;
	float wid;

	float ph12;
	float ph23;

	//complex<float> echo[10]; //存储复数信息用于时频分析

}CfarPoint;

#ifndef _RAWPOINT_
#define _RAWPOINT_

typedef struct _Rawpoint
{//点迹定位测角后的输出点迹，用于跟踪的输入

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

#endif

// 目标测角参数
typedef struct _AnglingPara
{

	bool is_sea;		// 0- sea or 1 - land;

	bool proc_method;	// 0-pd;   1-jump freq 

	float ant_d;		// 天线间距
	
	int win_az, win_ra; // 距离和方位向点迹凝聚窗大小
	float min_vr_was;

	int B_Na;
	long nr;

	float r_near;
	float dr;

	float prf;
	float az_vel;

	float scan_ang;
	float roll_ang;
	float pitch_ang;
	float yaw_ang;
	float fly_ang;

	float lati;
	float logn;
	float href;
	float aim_alt;
	
	float lookang;
	float lookside;
	float fdc;
	
	float east_v;
	float north_v;
	float up_v;

	int frame_no;
	int wave_no;

	float ph_ch1;
	float ph_ch2;
	
	float *pMainBornePhase;   // 主瓣中值相位


}AngPara;



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

struct rd_inv{

	double rn;
	double dr;
	double lambda;
	double fdc;

	long na;
	long nr;

	double lati;
	double lon;
	double high;
};


typedef struct tagUTMCorr 
{
	double x;
	double y;

}UTMCoor;

typedef struct tagWGS84Corr
{
	double lat;
	double log;

}WGS84Corr;

class AxisCal
{
public:

	AxisCal();

	~AxisCal();

	double sm_a;
	double sm_b;
	double sm_EccSquared;
	double UTMScaleFactor;

	// global variance 
	double scene_ref_lati;
	double scene_ref_logn;

	void WGS84ToENV(double lati, double logn, double href, double* Pos);

	void ENVToWGS84(double lati, double logn, double* Pos, double* latlogn);

	double RangeCalByGPS(double lati0,double logn0, double lati1, double logn1);

	double CalDirection(double lat_s,double lon_s, double lat_e, double lon_e);

	void WGS84ToScene(double lati,double logn, double* pos);					// should be improved

	void  ECEFToWGS84(double Px, double Py, double Pz, double lati_ref, double *latlogn);

	void WGS84ToECEF(double lati, double logn, double href, double *Pos);

	void SceneToWGS84(double Px, double Py, double lati_ref,double* latlogn);	// should be improved

	void ENVToECEF(double lati, double logn, double *P_env, double *P_ecef);

	void ECEFToENV(double lati, double logn, double Vx, double Vy, double Vz, double *Vout);

	void UTMxyToLatLon (double x, double y, int zone, bool southhemi, double *lati, double *lon);

	void LatLonToUTMxy (double lat, double lon, int *zone, double *UTMx, double *UTMy);

	// rtklib functions 
	__declspec(dllexport) void ecef2pos(const double *r, double *pos);

	__declspec(dllexport) void pos2ecef(const double *pos, double *r);

	__declspec(dllexport) void ecef2enu(const double *pos, const double *r, double *e);

	__declspec(dllexport) void enu2ecef(const double *pos, const double *e, double *r);

private:

	void  MapXYToLatLon (double x, double y, double lambda0, WGS84Corr &philambda);

	void MapLatLonToXY (double phi, double lambda, double lambda0, UTMCoor &xy);

	inline double DegToRad (double deg);

	inline double RadToDeg (double rad);

	double ArcLengthOfMeridian (double phi);

	inline double UTMCentralMeridian (int zone);

	double FootpointLatitude (double y);

	void matmul(const char *tr, int n, int k, int m, double alpha,const double *A, const double *B, double beta, double *CX);

	void tolocal(const double *pos, double *E);

	double dot(const double *a, const double *b, int n);

};

class Tarlocate {

public:

	Tarlocate();

	~Tarlocate();

	__declspec(dllexport) float GetAveAtitude(double lati_c, double logn_c, double span);
	__declspec(dllexport) void GetLocalDEMByCenterGPS(double lati_c, double lon_c);
	__declspec(dllexport) void GetHeight(double* curlati, double* curlon, double* curheight, __int64 num);

	__declspec(dllexport) void ArbitaryPosCal(double *pos_info, long *grid_info, long px, long py, double *latlogn);
	__declspec(dllexport) void Geo_locating(rd_par_input par_in, double *latlogn);
	__declspec(dllexport) void RD_locating(rd_par_input par_in, double *latlogn_rd, double *latlogn_geo);
	__declspec(dllexport) void RD_locating_ShortRange(rd_par_input par_in, double *latlogn_rd, double *latlogn_geo);
	__declspec(dllexport) bool RD_InvLoc(double *Vx, double *Vy, double *Vz, double *Px, double *Py, double *Pz, int na,rd_inv par,float *na_indx,float *nr_indx);
	__declspec(dllexport) double CalDirection(double lat_s,double lon_s, double lat_e, double lon_e);

private:

	double lati_center[DEM_CenterNum];				     //DEM数据各中心纬度

	double logn_center[DEM_CenterNum];				 //DEM数据各中心经度
	
	__int16 *dat_center;												//中心数据12k*12K

	int num_center;

	float res_LL_X, res_LL_Y, res_LU_X, res_LU_Y;
	
	float res_RL_X, res_RL_Y, res_RU_X, res_RU_Y;

	float BiLinInterp(float P_x0y0, float P_x1y0, float P_x0y1, float P_x1y1, float xcoef, float ycoef);
	
	float GetHeightIndex(double lati_img, double logn_img, double lati_center, double logn_center, float res_LU_X, int& flag);

	void ff(double x[DIM], double f[DIM], rd_par par);

	int gcpelim( int process, double A[DIM][DIM], double xx[DIM]);

	int secant2( double x[DIM], rd_par par);

	template <class T>
	inline T min_val(T *val,long num,__int64 *loc);
	
};


class MtiLocate {

public:
	
	MtiLocate();

	~MtiLocate();

	__declspec(dllexport) void MTIDotAngling_BichInterphase(vector<CfarPoint> cfarpoints, const AngPara & par, vector<RawPoint> & RawPoints);
	
	__declspec(dllexport) void MTIDotAngling_BichMainBornSearch(vector<CfarPoint> cfarpoints, const AngPara & par, vector<RawPoint> & RawPoints);

	__declspec(dllexport) void MTIDotAngling_TrichInterphase(vector<CfarPoint> cfarpoints, const AngPara & par, vector<RawPoint> & RawPoints);
	

private:

	int Point_Trace_1D(long *Tar_Ra, float *Tar_Amp, float *Tar_SNR, long Tar_Num, int win_ra);

	int Point_Trace(long *Tar_Az, long *Tar_Ra, float *Tar_Amp, float *Tar_ph, long Tar_Num, int win_ra, int win_az);
	
	int Point_Trace_Trich(long *Tar_Az, long *Tar_Ra, float *Tar_Amp, float *Tar_SNR12, float *Tar_SNR23, long Tar_Num, int win_ra, int win_az);

	template <class T>
	inline T min_val(T *val, long num, __int64 *loc);

};


#endif //