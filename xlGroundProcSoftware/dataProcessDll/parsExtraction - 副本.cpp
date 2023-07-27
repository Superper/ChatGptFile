#include "stdafx.h"
#include "parsExtraction.h"
#include <fstream>
#include <time.h>
#include <vector>
#include <Windows.h>
#include <math.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <Shlwapi.h>
#include <locale.h>

// for geo-tiff
#include "C:\MyLib\gdal-1.11_win32\GDAL1.11\include\gdal.h"
#include "C:\MyLib\gdal-1.11_win32\GDAL1.11\include\gdal_priv.h"

using namespace std;
using namespace RadarMtt;

#pragma comment(lib,"gdal_i.lib")
#pragma comment(lib,"ws2_32.lib")
#pragma comment(lib,"MulTarTrk.lib")

// 航迹关联类函数实现
struct mtitrack_para
{
	char mti_type; 
	NingjuPara njPa;
	TrackPara tkPa;
	KalManPara KmPa;

};

// global variance 
#define DOT_INFO_ITEM 16
#define TRK_INFO_ITEM 16
#define BURST_INFO_LEN 17920
MultiTragetsTrack g_mtt_obj_full;
vector<TrackPoint> *tk_out_pts;

unsigned char prev_ar[512];
float dotInfo[1024*DOT_INFO_ITEM];
float TrkInfo[1024*TRK_INFO_ITEM];
int Glb_DotNum;

		/// *************** 格式转换函数 *********************

		fmtConvCl::fmtConvCl(UINT8 * arGet)
		{
			ar=arGet;
		}

		fmtConvCl::~fmtConvCl()
		{
		}

		UINT16 fmtConvCl::byteToUint16(int start)
		{
				 for(int i=0;i<2;i++)
				{
					UInt16Un.u[i]=ar[start+i];
				}
				return UInt16Un.s;
		}

		__int16 fmtConvCl::byteToInt16(int start)
			 {
				 for(int i=0;i<2;i++)
				{
					Int16Un.u[i]=ar[start+i];
				}
				return Int16Un.s;
			 }
		//大端
		UINT16 fmtConvCl::b2U16BigEndian(int start)
		  {
			  UInt16Un.u[0]=ar[start+1];
			  UInt16Un.u[1]=ar[start];
			  return UInt16Un.s;
		  }
		  
		__int16 fmtConvCl::b216BigEndian(int start)
		  {
				
			    Int16Un.u[1]=ar[start];
				Int16Un.u[0]=ar[start+1];

				return Int16Un.s;
		  }

		UINT32 fmtConvCl::byteToUint32(int start)
			{
				for(int i=0;i<4;i++)
				{
					UInt32Un.u[i]=ar[start+i];
				}
				return UInt32Un.s;
			}

		__int32 fmtConvCl::byteToInt32(int start)
			{
				for(int i=0;i<4;i++)
				{
					Int32Un.u[i]=ar[start+i];
				}
				return Int32Un.s;
			}

		UINT32 fmtConvCl::b2U32BigEndian(int start)
		{
			UInt32Un.u[3]=ar[start];
			UInt32Un.u[2]=ar[start+1];
			UInt32Un.u[1]=ar[start+2];
			UInt32Un.u[0]=ar[start+3];

			return UInt32Un.s;
		}

		double fmtConvCl::byteToDouble(int start)
			 {
				 for(int i=0;i<8;i++)
				 {
					 db64Un.u[i]=ar[start+i];
				 }

				 return db64Un.s;
			 }

		float fmtConvCl::byteToFloat(int start)
			 {
				   for(int i=0;i<4;i++)
				 {
					 ft32Un.u[i]=ar[start+i];
				 }

				 return ft32Un.s;
			 }

		int   fmtConvCl::byteTo16Bcd(int start)
		{
			int retRes=0;

			//retRes+=ar[start+1]&0x0f;

			//retRes+=10*(ar[start+1]>>4);

			//retRes+=100*(ar[start]&0x0f);

			//retRes+=1000*(ar[start]>>4);

			retRes+=(ar[start+1]>>4);

			retRes+=10*(ar[start+1]&0x0f);

			retRes+=1000*(ar[start]&0x0f);

			retRes+=100*(ar[start]>>4);


			return retRes;
		}

		int   fmtConvCl::byteTo8Bcd(int start)
		{
			int retRes=0;

			retRes+=(ar[start]&0x0f);

			retRes+=10*(ar[start]>>4);

			return retRes;
		}

		double fmtConvCl::getResult(stParInFrame fmFrame)
		{
			double res=0;

			numTypeEnum type=fmFrame.numType;
			int st=fmFrame.startPosion;
			double unt=fmFrame.unit;

			if(type==uInt8Enm)
			{
				res=ar[st]*unt;
			}
			else if(type==uInt16Enm)
			{
				res=byteToUint16(st)*unt;
			}
			else if(type==int16Enm)
			{
				res=byteToInt16(st)*unt;
			}
			else if(type==uInt32Enm)
			{
				res=byteToUint32(st)*unt;
			}
			else if(type==int32Enm)
			{
				res=byteToInt32(st)*unt;
			}
			else if(type==fl32Enm)
			{
				res=byteToFloat(st)*unt;
			}
			else if(type==db64Enm)
			{
				res=byteToDouble(st)*unt;
			}
			else if(type==Bcd1Enm)
			{
				res=byteTo8Bcd(st)*unt;
			}
			else if(type==Bcd2Enm)
			{
				res=byteTo16Bcd(st)*unt;
			}
			return res;
	}

		double fmtConvCl::getBigEndianResult(stParInFrame fmFrame)
		{
			double res=0;

			numTypeEnum type=fmFrame.numType;
			int st=fmFrame.startPosion;
			double unt=fmFrame.unit;
			
			if(type==uInt8Enm)
			{
				res=ar[st]*unt;
			}
			else if(type==uInt16Enm)
			{
				res=b2U16BigEndian(st)*unt;
			}
			else if(type==int16Enm)
			{
				res=b216BigEndian(st)*unt;
			}
			else if(type==uInt32Enm)
			{
				res=b2U32BigEndian(st)*unt;
			}
			return res;
		}


		// ******** 图像产生及图像处理类函数实现 *********

		imgProc::imgProc(UINT8* arIn,string strPath)
		{
			rootPath=strPath+"\\";
			ar=arIn;
		}
		
		imgProc::imgProc()
		{
			rootPath="";
			ar=NULL;
		}

		int imgProc::norm_image(Mat & src){
			int type = src.type();
			//cout<<src(Range(1,10),Range(1,10))<<endl;
			if(type == CV_8U)
			{
				src.convertTo(src,CV_32F,1.0/255.0);
				return type;
			}
			else if(type == CV_16U)
			{
				src.convertTo(src,CV_32F,1.0/65535.0);
				return type;
			}
			else 
				return -1;
		}

		Mat imgProc::mean_filter(Mat & src,int WinW)
		{// OK 2016/06/02

			Mat mean_kernel = Mat::ones(WinW*2+1,WinW*2+1,CV_32F);
			Mat image_mean;
			mean_kernel = mean_kernel * (1.0/((WinW*2+1)*(WinW*2+1)));
			filter2D(src,image_mean,-1, mean_kernel);
			return image_mean;
		}

		Mat imgProc::var_filter(Mat & src, int WinW)
		{// OK 2016/06/02

			Mat src2 = src.mul(src);
			src2 = mean_filter(src2,WinW);
			Mat src_mean = mean_filter(src,WinW);
			Mat var = src2 - src_mean.mul(src_mean);
			return var;
		}

		Mat imgProc::Lee(Mat& image_in,int WinW,float Looks)
		{// OK 2016/06/02

			Mat dst = image_in.mul(image_in);
			float noise_var = 1/sqrtf(Looks);
			//参数提取
			Mat I_mean = mean_filter(dst,WinW);
			Mat I_var = var_filter(dst,WinW);
			Mat x_var = (I_var -I_mean.mul(I_mean) * noise_var)/(1-noise_var);
			
			//滤波
			double var_min,var_max,cmin,cmax; 
			minMaxLoc(I_var, &var_min,&var_max);
			cmin = var_min + 0.1*(var_max - var_min);
			cmax = var_min + 0.9*(var_max - var_min);
			Mat ImgX = x_var.mul(1.0/I_var);
			Mat temp0=dst-I_mean;
			Mat temp1=ImgX.mul(temp0);
			ImgX = I_mean + temp1;
	
			float *p_var = (float *) I_var.data;
			float *p_dst = (float *) dst.data;
			float *p_x = (float *) ImgX.data;
			float *p_m = (float *) I_mean.data;
			float *temp;
	
			long long Num = dst.cols *dst.rows;

			do{
				if(*p_var<cmin)
					*p_x = *p_m;
				else if(*p_var>cmax)
					*p_x = *p_dst;
				p_x ++;
				p_m ++;
				p_dst ++;
				p_var ++;
			}while(--Num);
	
			sqrt(ImgX,ImgX);
			return ImgX;
		}

		Mat imgProc::Gamma(Mat& image_in,int WinW,float Looks)
		{
			Mat dst;
			image_in.copyTo(dst);
			float noise_var = 1/sqrtf(Looks);
			//参数提取
			Mat I_mean = mean_filter(dst,WinW);
			Mat I_var = var_filter(dst,WinW);
			Mat c_i;
			sqrt(I_var,c_i);
			c_i = c_i.mul(1.0/I_mean);
			Mat a = (1+noise_var*noise_var)/(c_i.mul(c_i) -noise_var*noise_var);
			Mat middle = a - Looks -1;
			middle = middle.mul(I_mean);

			float *p_var = (float *) I_var.data;
			float *p_dst = (float *) dst.data;
			float *p_c_i = (float *) c_i.data;
			float *p_middle = (float *) middle.data;
			float *p_mean = (float *) I_mean.data;
			float *p_a = (float *) a.data;
			float *p_in = (float *) image_in.data;
			float temp;
			long long Num = dst.cols *dst.rows;

			do{
				if(((*p_c_i)>= noise_var) && ((*p_middle) >=0) && ((*p_c_i) <= 1.414*noise_var))
				{
					temp = sqrtf((*p_middle)*(*p_middle) + 4*Looks*(*p_a) * (*p_in)*(*p_mean));
					*p_dst = ((*p_middle) + temp)/(2*(*p_a));
				}
				else if(*p_c_i < noise_var)
				{
					*p_dst = *p_mean;
				}
				else if(*p_c_i > 1.414*noise_var)
				{
					*p_dst = *p_in;
				}
				p_var++;
				p_dst++;
				p_c_i++;
				p_middle++;
				p_mean++;
				p_a++;
				p_in++;
			}while(--Num);
			return dst;
		}

		Mat imgProc::Kuan(Mat& image_in,int WinW,float Looks)
		{
			Mat dst;
			image_in.copyTo(dst);
			float noise_var = 1/sqrtf(Looks);
			//参数提取
			Mat I_mean = mean_filter(dst,WinW);
			Mat I_var = var_filter(dst,WinW);
			Mat x_var = (I_var - I_mean.mul(I_mean)*(noise_var*noise_var))/(1+noise_var*noise_var);
			Mat tempM = x_var>0;
			tempM.convertTo(tempM,CV_32F,1.0/255);
			x_var = x_var.mul(tempM);

			tempM = x_var.mul(1/(I_mean.mul(I_mean) + x_var)*(noise_var*noise_var));
			dst = I_mean + tempM.mul(image_in - I_mean);
			return dst;
		}

		Mat imgProc::Frost(Mat& image_in,int WinW)
		{
			//Mat dst;
			//image_in.copyTo(dst);
			//int M = WinW*2+1;
			//float p = 0.1;
			Mat I_mean = mean_filter(image_in,WinW);
			//Mat I_var = var_filter(image_in,WinW);
			
			return I_mean;
		}

		Mat imgProc::Sigma(Mat& image_in,int WinW)
		{
			Mat dst;
			image_in.copyTo(dst);
			int M = WinW*2+1;
			float sigma = 0.3;
			Mat subImage = Mat::ones(M,M,CV_32F);
			Mat tmp, tmp2;
			Scalar Sum_tmp;

			for(int ci=WinW; ci<image_in.cols - WinW-1;ci++)
				for(int ri = WinW; ri<image_in.rows - WinW-1;ri++)
				{
					subImage = image_in(Range(ri-WinW,ri+WinW+1),Range(ci-WinW,ci+WinW+1));
					tmp = (subImage >= image_in.at<float>(ri,ci) * (1-2*sigma))+(subImage <= image_in.at<float>(ri,ci) * (1+2*sigma));
			
					tmp.convertTo(tmp,CV_32F,1.0/255);
			
					Sum_tmp = sum(subImage.mul(tmp))/sum(tmp);
					dst.at<float>(ri,ci) = Sum_tmp.val[0];
				}
				return dst;
		}

		void imgProc::ImgScale(uchar *image_in, int &width, int &height, float width_fscale, float height_fscale)
		{
			// 图像如果放大则需要另行开辟内存空间
			__int64 kk,jj;
			__int64 M = __int64(width)*height;

			int width_out  = int(width*width_fscale/4.0)*4;
			int height_out = int(height*height_fscale/4.0)*4;

			IplImage  *image1 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1);
			IplImage  *image2 = cvCreateImage(cvSize(width_out,height_out),IPL_DEPTH_8U,1);
	
			memcpy(image1->imageData,image_in,sizeof(uchar)*M);
	
			cvResize(image1,image2,CV_INTER_AREA);

			M = __int64(width_out)*height_out;
			memcpy(image_in,image2->imageData,sizeof(uchar)*M);

			width  = width_out;
			height = height_out;

			cvReleaseImage(&image1);
			cvReleaseImage(&image2);
	
		}

		// 图像任意角度旋转 check ok
		void imgProc::ImgRotate(uchar *image_in, int width, int height, int angle,Mat& mat_out)
		{
			double scale = 1.0;
			long width_new  = long((height*fabs(sin(angle*CV_PI/180))+width*fabs(cos(angle*CV_PI/180))+1)/4.0)*4;
			long height_new = long((height*fabs(cos(angle*CV_PI/180))+width*fabs(sin(angle*CV_PI/180))+1)/4.0)*4;

			Point center = Point(width/2,height/2);
			Mat rot_mat = getRotationMatrix2D(center,angle,scale);

			rot_mat.at<double>(0,2) += (width_new - width)/2;
			rot_mat.at<double>(1,2) += (height_new - height)/2;

			Mat mat_in;
			mat_in.data = image_in;
			mat_in.rows = height;
			mat_in.cols = width;

			warpAffine(mat_in,mat_out,rot_mat,cv::Size(width_new,height_new),1,0,0);

			//imwrite("d:\\123.tif",res_mat);

		}

		void imgProc::gamma_correct(Mat &src,float high,float low, float gamma)//src 单通道，CV_8U或者CV_16U
		{
	
			float paramA,paramB;
			paramA = 1.0/(high-low);
			paramB = -paramA *low;

			int hist_bin_num ;
			int type = src.type();
			if(type == CV_8U)//规范化到0-1之间的float类型
				hist_bin_num = 256;
			if(type == CV_16U)
				hist_bin_num = 65536;
			Mat lut(hist_bin_num,1,CV_32F);
			lut.setTo(0);
			for(int i = 0;i<hist_bin_num;i++)
			{
				if(i<=low*(hist_bin_num-1))
					lut.at<float>(i) = 0;
				else if(i>=high*(hist_bin_num-1))
					lut.at<float>(i) = 1;
				else
				{
					lut.at<float>(i) = powf(paramA*(float)i/(hist_bin_num-1)+paramB,gamma);
				}
			}
			lut.convertTo(lut,CV_8U,255);
	
			unsigned long long *p =(unsigned long long *) src.data, temp;
			unsigned char * pl = (unsigned char *) lut.data;
	
			if(type == CV_16U)
			{
				long long NUM = src.cols*src.rows/4;
				unsigned long long D0,D1,D2,D3;
				do 
				{ 
					temp = *p;
					D0 = *(pl+(temp & 0xffff));
					D1 = *(pl+((temp>>16) & 0xffff));
					D2 = *(pl+((temp>>32) & 0xffff));
					D3 = *(pl+((temp>>48) & 0xffff));
					*(p++) = (D3<<48) | (D2<<32) | (D1<<16) | D0;
				} while (--NUM);
				src = src*256;
				//src.convertTo(src,CV_8U);
			}
			if(type == CV_8U)
			{
				long long NUM = src.cols*src.rows/8;
				unsigned long long D0,D1,D2,D3,D4,D5,D6,D7;
				do 
				{ 
					temp = *p;
					D0 = *(pl+(temp & 0xff));
					D1 = *(pl+((temp>>8) & 0xff));
					D2 = *(pl+((temp>>16) & 0xff));
					D3 = *(pl+((temp>>24) & 0xff));
					D4 = *(pl+((temp>>32) & 0xff));
					D5 = *(pl+((temp>>40) & 0xff));
					D6 = *(pl+((temp>>48) & 0xff));
					D7 = *(pl+((temp>>56) & 0xff));
					*(p++) = (D7<<56) | (D6<<48) | (D5<<40) | (D4<<32) |
							(D3<<24) | (D2<<16) | (D1<<8) | D0;
				} while (--NUM);
			}
		}

		void imgProc::high_low(Mat &src ,float &low,float &high)
		{
				//设定bin数目
			int histSize = 256;
			float range[] = { 0,255 };
			const float* histRange = { range };
			bool uniform = true; 
			bool accumulate = false;

			Mat NormSrc,hist;
	
			//设置ROI用于计算直方图
			int HeightBlockNum(0),WidthBlockNum(0),BlockHeight(256),BlockWidth(256);
			int jumpBlock;
			int BlockLength = MIN(src.rows,src.cols)/BlockHeight;
			if(BlockLength < 3)
				jumpBlock = 1;
			else if(BlockLength < 6)
				jumpBlock = 3;
			else if(BlockLength < 12)
				jumpBlock = 5;
			else if(BlockLength < 18)
				jumpBlock = 6;
			else if(BlockLength < 40)
				jumpBlock = 8;
			else if(BlockLength < 60)
				jumpBlock = 10;
			else 
				jumpBlock = 12;
	
			Rect roi = Rect(0,0,BlockWidth,BlockHeight);
			HeightBlockNum = src.rows /(BlockHeight*(jumpBlock));
			WidthBlockNum = src.cols /(BlockWidth*(jumpBlock));
			for(int Hidx = 0; Hidx < HeightBlockNum;Hidx++ )
			{
				for (int Widx = 0; Widx < WidthBlockNum; Widx++)
				{
					roi.x = Widx * jumpBlock * BlockWidth;
					roi.y = Hidx * jumpBlock * BlockWidth;
					NormSrc = src(roi);
					if(src.type() == CV_16U)//16位数据
					{	
						NormSrc.convertTo(NormSrc,CV_8U,1.0/256);
					}
					if(Hidx == 0 && Widx ==0)
					{
						calcHist( &NormSrc, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );
						accumulate = true;
					}else
					{
						calcHist( &NormSrc, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );
					}
				}
			}
	
			{//计算高低值
				Mat cdf;
				double minVal, maxVal,sc;
				int tempIdx[]={0,0};

				Mat temp;
				hist.convertTo(hist,CV_32F,(1.0/norm(hist,NORM_L1)));//归一化
				hist.copyTo(cdf);
	
				for(int i = 1;i<cdf.rows;i++)
					cdf.at<float>(i) = cdf.at<float>(i-1) + cdf.at<float>(i);//计算累积分布
	
				absdiff(cdf,low,temp);
				minMaxIdx(temp,& minVal, & maxVal,tempIdx);
				low = ((float)tempIdx[0])/(float)cdf.rows;
	   
				absdiff(cdf,high,temp);
				minMaxIdx(temp,& minVal, & maxVal,tempIdx);
				high = ((float)tempIdx[0])/(float)cdf.rows;
			}
		}

		void imgProc::SAR_VS_params_init(SAR_VS_params &params)
		{
			params.input_file_name="";
			params.output_file_name="";
			params.tifParams.dataMat=NULL;
			params.tifParams.dtHeight=0;
			params.tifParams.dtWidth=0;
			params.WinWidth = 2;
			params.looks = 4;
			params.gamma = 1;
			params.low = 0.01;
			params.high = 0.98;
			params.funtype = params.AUTO;
		}
		
		void imgProc::Local_Funcs(SAR_VS_params param)
		{
			if(param.funtype == param.AUTO)
			{
				Mat src = imread(param.input_file_name,-1);
				int type = src.type();
				float high, low;
				high = param.high;
				low = param.low;
				high_low(src,low,high);
				gamma_correct(src,high,low,param.gamma);//src 单通道，CV_8U或者CV_16U
				imwrite(param.output_file_name,src);
			}

			if(param.funtype == param.LEE)
			{
				Mat src = imread(param.input_file_name,-1);
				int type = src.type();
				norm_image(src);
				src = Lee(src,param.WinWidth,param.looks);
				src.convertTo(src,type,(type==CV_8U) ? (255.0) : (65535.0));
				imwrite(param.output_file_name,src);
			}
			
			if(param.funtype == param.FROST)
			{
				Mat src = imread(param.input_file_name,-1);
				int type = src.type();
				norm_image(src);
				src = Frost(src,param.WinWidth);
				src.convertTo(src,type,(type==CV_8U) ? (255.0) : (65535.0));
				imwrite(param.output_file_name,src);
			}

			if(param.funtype == param.GAMMA)
			{
				Mat src = imread(param.input_file_name,-1);
				int type = src.type();
				norm_image(src);
				src = Gamma(src,param.WinWidth,param.looks);
				src.convertTo(src,type,(type==CV_8U) ? (255.0) : (65535.0));
				imwrite(param.output_file_name,src);
			}

			if(param.funtype == param.KUAN)
			{
				Mat src = imread(param.input_file_name,-1);
				int type = src.type();
				norm_image(src);
				src = Kuan(src,param.WinWidth,param.looks);
				src.convertTo(src,type,(type==CV_8U) ? (255.0) : (65535.0));
				imwrite(param.output_file_name,src);
			}

			if(param.funtype == param.SIGMA)
			{
				Mat src = imread(param.input_file_name,-1);
				int type = src.type();
				norm_image(src);
				src = Sigma(src,param.WinWidth);
				src.convertTo(src,type,(type==CV_8U) ? (255.0) : (65535.0));
				imwrite(param.output_file_name,src);
			}
		}

		bool imgProc::Mirror_Judge(unsigned char* data)
		{
			commonFuncs cmFuncs;
			muInfo mu(data);
			
			bool be_mirror = false;

			int year = mu.date_year;
			int month = mu.date_month;
			int day = mu.date_day;

			int PowerOnTime = cmFuncs.getPowerOnTime(data);
			
			if(year%2000 == 16 && month == 8 && day == 23)
			{
				if(PowerOnTime == 9 || PowerOnTime == 11 || PowerOnTime == 16)be_mirror = true;
			}

			return be_mirror;

		}

		void imgProc::Gray_adjust(unsigned __int8 *pdata, __int64 len,__int64 hist_sum_th_ratio)
		{
			// 数据智能量化； Dev by Yunhua-Luo @ 2016/03/23
			
			if(hist_sum_th_ratio <= 0 || hist_sum_th_ratio > 100) return;

			__int64 hist_sum_th = __int64(len*0.0001*hist_sum_th_ratio);
			__int64 hist_up	    = 255;
			__int64 k;
			double  rat;
	
			// 1. 求解直方图
			__int64 *hist      = new __int64[256];
			__int64 *hist_sum  = new __int64[256];
			memset(hist,0,sizeof(__int64)*256);
			memset(hist_sum,0,sizeof(__int64)*256);

			for(k=0; k<len; k++) hist[pdata[k]]++;
			hist_sum[255] = hist[255];
			for(k=255; k>0; k--) hist_sum[k-1] = hist_sum[k]+hist[k-1];

			// 2. 干掉最大部分
			for(k=255; k>0; k--)
			{
				if(hist_sum[k] > hist_sum_th){ hist_up = k;break;}
			}
			delete [] hist;
			delete [] hist_sum;
		
			// 4. 数据重新量化；
			__int64 level = 255;
			rat = double(level)/hist_up;
	
			#pragma omp parallel for
			for(k=0; k<len; k++) 
			{				
				if(pdata[k] >= hist_up)   
					pdata[k] = 255;
				else 
					pdata[k] = (unsigned __int8)(rat*pdata[k]);
			}
		}
		
		void imgProc::Gray_AlogQuan(unsigned __int8 *pdata, __int64 len)
		{
			float aveP = 0; 
			float maxP;
			for (long k = 0; k < len; k++)
				aveP += pdata[k] * pdata[k];
			aveP /= len;

			aveP = 10 * log10(aveP);
			long loc;
			double aa = max_val(pdata, len, &loc);
			maxP = 10 * log10(pdata[loc] * pdata[loc]);
			float coef = 1.0;
			float ave_max = maxP - aveP;
			if (ave_max < 10) coef = 1.0;
			if (ave_max > 10 && ave_max <= 15) coef = 0.97;
			if (ave_max > 15 && ave_max <= 20) coef = 0.92;
			if (ave_max > 20 && ave_max <= 25) coef = 0.87;
			if (ave_max > 25 && ave_max <= 30) coef = 0.82;
			if (ave_max > 30 && ave_max <= 35) coef = 0.77;
			if (ave_max > 35 && ave_max <= 40) coef = 0.72;
			if (ave_max > 40) coef = 0.67;
		
			float coef1 = log2(maxP / (coef*aveP));
			#pragma omp parallel for
			for (long k = 0; k < len; k++)
				pdata[k] = unsigned __int8(coef1*log2(1+log2(pdata[k])/(coef*maxP))*255);

		}


		void imgProc::ImageCorr_for6suoMap(unsigned char *pdata, __int64 row, __int64 col, int lookside)
		{
			// 图像校正为了适应六所的要求对图像进行旋转

			// *** image rotate ****
			Mat src(row,col,CV_8UC1);
			memcpy(src.data,pdata,sizeof(unsigned char)*row*col);
			src.rows = row;
			src.cols = col;
			src.dims = 2;
		
			// *** image rotate ****
			flip(src,src,0);			 // image rotate  == 0 vertical rotate // < 0 180 rotate
			if(lookside == LEFT_SIDE)	 // left side with horizontal rotate 
			flip(src,src,1);			 // left-right rotate
			
			memcpy(pdata,src.data,sizeof(unsigned char)*row*col);

		}

		// Added by Yunhua-Luo 2021/07/12
		void imgProc::Local_Funcs_Lookside(SAR_VS_params param, int lookside, double *gps_info)
		{

			int dtH=param.tifParams.dtHeight/10; // create point address
			int dtW=param.tifParams.dtWidth;

			Mat src(dtH,dtW,CV_8UC1);
			src.data=param.tifParams.dataMat;
			src.rows=param.tifParams.dtHeight;
			src.cols=param.tifParams.dtWidth;
			src.dims=2;

			// Image correction  
			ImageCorr_for6suoMap(src.data,src.rows,src.cols,lookside);

			// *** gray adjust by luo ****
			// Read parameter file @ added by Yunhua-Luo @ 2016/3/23
			FILE* fip;
			__int64 hist_sum_th = 0;  // 0--100;
			char para_str[128];
			if ((fip = fopen("gray_par.txt", "r")) != NULL)
			{
				if (fscanf(fip, "%s", para_str) != EOF)
					hist_sum_th = __int64(atoi(para_str));
				fclose(fip);
			}
			commonFuncs cmFuncs;
			cmFuncs.logRecords("gray adjust hist_sum_th: ", double(hist_sum_th));

			__int64 data_len = __int64(src.cols)*__int64(src.rows);
			Gray_adjust(src.data, data_len, hist_sum_th);	// my gray_adjust;

			cmFuncs.logRecords("Rawcols: ",double(src.cols));
			cmFuncs.logRecords("Rawrows: ",double(src.rows));
			cmFuncs.logRecords("az_ml2_coef: ",double(az_ml2_coef));

			if(ACCURATE_ML && az_ml2_coef != 1)
			{
				if(az_ml2_coef < 1) 
					ImgScale(src.data,src.cols,src.rows,1,az_ml2_coef);
				else
					ImgScale(src.data,src.cols,src.rows,1.0/az_ml2_coef,1);
			}

			cmFuncs.logRecords("Adjustcols: ",double(src.cols));
			cmFuncs.logRecords("Adjustrows: ",double(src.rows));
			cmFuncs.logRecords("az_ml2_coef: ",double(az_ml2_coef));
			
			// ***** write out tif file ******
			if(!GEOTIFF_VERSION)
			{
				cmFuncs.logRecords("ImageWrite", double(az_ml2_coef));
				imwrite(param.output_file_name,src);
			}else
			{
				cmFuncs.logRecords("GeoTiffWrite", double(az_ml2_coef));
				GeoTiff geo;
				geo.WriteByte2Geotiff(gps_info,gps_info+4,src.cols,src.rows,src.data,param.output_file_name.c_str());
			}
		}

		// Added by Robles 2022/01/05
		int  imgProc::creatCoTif_New(UINT8* dataBuffer,__int64 data_len)
		{

			#define AUX_DATA_SIZE     512
			#define SUB_IMAGE_HEIGHT  512
			#define MAX_IMAGE_WIDTH   32768

			bool bDumpBuffer   = false;
			bool bResetLoopNum = false;

			static int blcokRecvCnt = -1;  // 初次调用
			static int loopNum = 0;
			static unsigned char *imgBuffer = NULL;
			static unsigned char *parBuffer = NULL;

			static string outputPath;

			int lastPowerOnTime, currentPowerOnTime;

			commonFuncs cmFuncs;
			int numToMerge;

			int loopNum1 = cmFuncs.getLoopNum(dataBuffer);	// 获取本次拼接图像的周期号

			cmFuncs.logRecords("loopNum: ",double(loopNum1));
			cmFuncs.logRecords("dump: ",double(bDumpBuffer));

			int mode;
		    
			if(blcokRecvCnt < 0) // 初次调用
			{	
				blcokRecvCnt = 0;

				loopNum = cmFuncs.getLoopNum(dataBuffer);	// 获取本次拼接图像的周期号
				
				mode = cmFuncs.getSarModel(dataBuffer);

				if(mode == 3) 
				{   // only SAR-GMTI mode
					int mView = cmFuncs.getMViewNum(dataBuffer);
					numToMerge = int(64 / mView);

				}else 
				{
					numToMerge = 8;  
				}
			}else
			{

				mode = cmFuncs.getSarModel(parBuffer);
				if(mode == 3)
				{  // only SAR-GMTI mode
					int mView = cmFuncs.getMViewNum(parBuffer);
					numToMerge = int(64 / mView);
				}else 
					numToMerge = 8; 

				lastPowerOnTime    = cmFuncs.getPowerOnTime(parBuffer);
				currentPowerOnTime = cmFuncs.getPowerOnTime(dataBuffer);
				if(currentPowerOnTime != lastPowerOnTime) bDumpBuffer = true;
			}

			if(blcokRecvCnt == numToMerge) 
				bDumpBuffer = true;
			else 
				bDumpBuffer = false;

			if(bDumpBuffer)  //存满后输出TIFF图像;
			{	

				loopNum = cmFuncs.getLoopNum(parBuffer); // new version 

				// 需要从缓存中把图像及参数输出
				string flPath = outputPath.append(cmFuncs.getCombPicName(parBuffer, loopNum) + ".tif");
        
				// 参数文件名
				int pathLen = flPath.length();	
				char *nameCh = new char[pathLen + 1];
				for(int i = 0; i < pathLen; i ++)
				{
					nameCh[i] = flPath[i];
				}
				nameCh[pathLen - 3] = 't';
				nameCh[pathLen - 2] = 'x';
				nameCh[pathLen - 1] = 't';
				nameCh[pathLen] = '\0';
        
				FILE *FL = fopen(nameCh, "a");
				delete [] nameCh;
				if(FL == NULL)return ERROR_PATH;
				
				muInfo mu0(parBuffer);

				sarImageInfo imgInfo(parBuffer);
				sarImageInfo imgInfo_end(parBuffer + (blcokRecvCnt - 1) * AUX_DATA_SIZE);
				
				// ******* 模式判断，数据截断  Added by Robles 2021/08/18 ******
				if (imgInfo.work_mode >=1 && imgInfo.work_mode <=2 && fabs(imgInfo.azAngle-imgInfo_end.azAngle)<0.01)
				{
					return -1; 
				}

				imgInfo.img_rows = blcokRecvCnt * SUB_IMAGE_HEIGHT;	
				imgInfo.img_length *= blcokRecvCnt;
				imgInfo.ml2_coef = 1.0;

				// 修正GPS定位参数信息 2016/3/20 by Luo
				imgInfo_end.plane_latitude    += ((imgInfo_end.plane_latitude - imgInfo.plane_latitude)/(blcokRecvCnt - 1));

				imgInfo_end.plane_longitude += ((imgInfo_end.plane_longitude- imgInfo.plane_longitude)/(blcokRecvCnt - 1));

				imgInfo.sarImageLocating(imgInfo.plane_latitude,imgInfo.plane_longitude,imgInfo_end.plane_latitude,imgInfo_end.plane_longitude);
    			
				double gps_info[8];
				memset(gps_info,0,sizeof(double)*8);

				// 点顺序： 左上、右上、左下、右下
				// ******* for geo-tiff generation ********
				gps_info[0] = imgInfo.left_top_latitude;
				gps_info[1] = imgInfo.right_up_latitude;
				gps_info[2] = imgInfo.left_down_latitude;
				gps_info[3] = imgInfo.right_down_latitude;

				gps_info[4] = imgInfo.left_top_longitude;
				gps_info[5] = imgInfo.right_up_longitude;
				gps_info[6] = imgInfo.left_down_longitude;
				gps_info[7] = imgInfo.right_down_longitude;

				taskInfo task0(parBuffer);
				
				// 输出任务参数
				fprintf(FL, "程序版本: V4.220107\n");
				task0.output(FL);

				imgInfo.ml2_coef = az_ml2_coef;

				imgInfo.output(FL);			

				cmFuncs.OutputRDLocatingInfo(FL, &imgInfo, &imgInfo_end); // New Added by Robles 2022/01/06

				mu0.output(FL,0);  // write file 
    
				fprintf(FL,"\n\n\n");
    
				// 输出每个子图的信息
				int add_val =1;
				for(int j = 0; j < blcokRecvCnt; j ++)
				{ 
					fprintf(FL, "子图序号:%d\n", j + 1);
					taskInfo task(parBuffer + j * AUX_DATA_SIZE);
					task.output(FL);

					sarImageInfo si1(parBuffer + j * AUX_DATA_SIZE);

					if(j==(blcokRecvCnt-1)) 
						add_val = 0;
					else 
						add_val = 1;
					
					sarImageInfo si2(parBuffer + (j+add_val) * AUX_DATA_SIZE);
					muInfo mu(parBuffer + j * AUX_DATA_SIZE);

					si1.sarImageLocating(si1.plane_latitude,si1.plane_longitude,si2.plane_latitude,si2.plane_longitude);
					
					si1.ml2_coef = az_ml2_coef;
					si1.output(FL);
					
					mu.output(FL,j);
					fprintf(FL, "\n");
				}
				fclose(FL);

				// ************** 输出图像 *********************
				int tifWidth  = cmFuncs.getRangePoints(parBuffer);
				int tifHeight = blcokRecvCnt * SUB_IMAGE_HEIGHT;

				// 图像可视化增强
				SAR_VS_params stPars;
				SAR_VS_params_init(stPars);
				stPars.output_file_name = flPath;
				stPars.tifParams.dataMat = imgBuffer;
				stPars.tifParams.dtWidth = tifWidth;
				stPars.tifParams.dtHeight = tifHeight;

				// Output image V2021/07/12
				sarImageInfo myInfo(parBuffer);
				Local_Funcs_Lookside(stPars, myInfo.look_Side,gps_info); 
        
				// 清理缓存
				delete [] imgBuffer;
				imgBuffer = NULL;
				delete [] parBuffer;
				parBuffer = NULL;

				blcokRecvCnt = 0;
			}


			// Added by Yunhua-Luo 2017/8/15
			int temploopNum = cmFuncs.getLoopNum(dataBuffer); // new version 
			if(temploopNum != loopNum)  // 如何周期号不全，则直接去掉真整个图像
			{
				loopNum = temploopNum; // 重新获取本次的拼接的周期号
				blcokRecvCnt = 0;
			}

			int tifWidth = cmFuncs.getRangePoints(dataBuffer);
    
			// 计数器为0, 建立缓存
			if(blcokRecvCnt == 0)
			{
				if(imgBuffer == NULL)
				{
					imgBuffer = new unsigned char[tifWidth * SUB_IMAGE_HEIGHT * numToMerge];  // 最多8个子图
					memset(imgBuffer,0,sizeof(unsigned char)*__int64(tifWidth) * SUB_IMAGE_HEIGHT * numToMerge);
				}
					
				if(parBuffer == NULL)	parBuffer = new unsigned char[AUX_DATA_SIZE * numToMerge];
				
				outputPath = rootPath;
			}
			
			// 将当前子图及辅助数据压入缓存  // Modified by Yunhua-Luo 2017/09/01;
			memcpy(imgBuffer + blcokRecvCnt * tifWidth * SUB_IMAGE_HEIGHT, dataBuffer + AUX_DATA_SIZE, data_len - AUX_DATA_SIZE);
			memcpy(parBuffer + blcokRecvCnt * AUX_DATA_SIZE, dataBuffer, AUX_DATA_SIZE);
			blcokRecvCnt ++;  
    
			return 0;
		}

		// main proprocess function 
		void imgProc::allImageProMethod(char *strImage,char *strDest,imageProcParsST procPars)
		{// check 06/03

			string flSrcPath(strImage);
			string flDestPath(strDest);
			Mat src;

			double adfGeoTransform[12];
			long heightI = 8192;
			long widthI  = 32768;
			UINT8 *dataR = new UINT8[heightI*widthI];
			memset(dataR,0,sizeof(UINT8)*heightI*widthI);

			if(GEOTIFF_VERSION)
			{
				// input strImage is geo-tiff image 
				GeoTiff geo;
				long info[2];
				geo.Read2Geotiff(strImage,dataR,info,adfGeoTransform);

				widthI  = info[0];
				heightI = info[1];

				if(dataR[0] = 0 || widthI < 0)
				{
					printf("image is not geo-tiff format !!!\n");
					return;
				}

			}else
			{
				src = imread(flSrcPath,-1);
				widthI  = src.cols;
				heightI = src.rows;
				
				const int typeI=src.type();
				for(long i=0;i<heightI*widthI;i++)
				{
					dataR[i]= src.data[i];
				}
			}

			unsigned char *dataRes=new unsigned char[heightI*widthI];
			memcpy(dataRes,dataR,sizeof(unsigned char)*heightI*widthI); // by luo

			__int64 img_nr = widthI;
			__int64 img_na = heightI; // 图像的大小
			float   dr     = procPars.ra_res;	  // 距离采样间隔
			float   r_near = procPars.r_near; // 近端斜距；
			float   href   = procPars.href;   // 飞机到地面高度
			float   r_ang  = procPars.r_ang*PI/180.0;  // 雷达下视角;
			float   ra_beam_ang = procPars.ra_beam_ang*PI/180.0; // 距离向波束宽度;
			float heading_angle = procPars.heading_angle;
			unsigned char *img_data = dataRes; // 存储数据
			int multilook = 2;

			// ******* 辐射校正 *********
			if(procPars.blAmpCor>0)
			{
				float *ampl_coef1 = new float[img_nr];
				memset(ampl_coef1,1,sizeof(float)*img_nr);
				cal_ampl_coef(ampl_coef1,dr,r_near,href,r_ang,ra_beam_ang,img_nr);
				ampl_corr(img_data,ampl_coef1,img_na,img_nr);		
				delete [] ampl_coef1;
			}

			// ******** 几何校正 ********
			if(procPars.blGeoCor>0)
			{
				geo_corr(img_data,dr,r_near,href,img_na,img_nr);
			}

			// *** Remove speckle ****
			if(procPars.blRemoveSpots>0)
			{

				smooth(img_data,img_na,img_nr);

				//Lee(src,3,multilook);
				//Frost(src,6);
			}


			src.data = img_data;
			src.cols = widthI;
			src.rows = heightI;
				

			// *** equal histgram ****
			if(procPars.blHs>0)
			{
				long len = heightI*widthI;
				Gray_adjust(src.data,len,99);
			}

			// ****** Rotate to North *********
			if(0)
			{
		
				int angle = 60; //360-heading_angle; // 逆时针旋转航向角
				double scale = 1.0;
				long width_new  = long((heightI*fabs(sin(angle*CV_PI/180))+widthI*fabs(cos(angle*CV_PI/180))+1)/4.0)*4;
				long height_new = long((heightI*fabs(cos(angle*CV_PI/180))+widthI*fabs(sin(angle*CV_PI/180))+1)/4.0)*4;

				Point center = Point(widthI/2,heightI/2);
				Mat rot_mat = getRotationMatrix2D(center,angle,scale);

				rot_mat.at<double>(0,2) += (width_new - widthI)/2;
				rot_mat.at<double>(1,2) += (height_new - heightI)/2;

				//uchar *output = new uchar[width_new*height_new];
				//memset(output,0,sizeof(uchar)*width_new*height_new);
				//if(output == NULL) return;

				Mat mat_out;
				//mat_out.data = output;
				//mat_out.cols = width_new;
				//mat_out.rows = height_new;
				
				warpAffine(src,mat_out,rot_mat,cv::Size(width_new,height_new),1,0,0);

				if(GEOTIFF_VERSION)
				{	
					GeoTiff geo;
					char *gcs_wgs1984="GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.01745329251994328,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]]";
					geo.GeotiffWriteFromByteData(mat_out.data,mat_out.rows,mat_out.cols,strDest,adfGeoTransform,gcs_wgs1984); //调用下面的写geotiff函数
				
					//imwrite(flDestPath,mat_out);
				}else
				{
					imwrite(flDestPath,mat_out);
				}

				//delete [] output;  output  = NULL;
				delete [] dataRes; dataRes = NULL;
				delete [] dataR;   dataR   = NULL;

				return;
			}


			// ***** save image to geotif *********

			if(GEOTIFF_VERSION)
			{	
				GeoTiff geo;
				char *gcs_wgs1984="GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.01745329251994328,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]]";
				geo.GeotiffWriteFromByteData(src.data,src.rows,src.cols,strDest,adfGeoTransform,gcs_wgs1984); //调用下面的写geotiff函数
			
			}else
			{
				imwrite(flDestPath,src);
			}

			delete [] dataRes; dataRes = NULL;
			delete [] dataR;   dataR   = NULL;

		}

		double imgProc::sinc(double x)
		{
			double  val;
			if(x==0)  val=1.0;
			else      val=sin(PI*x)/x/PI;
			return val;
		}
		
		void imgProc::cal_ampl_coef(float *ampl_coef,float dr,float r_near ,float href,float r_ang, float ra_beam_ang,long nr )
		{// OK 
		 // ra_beam_ang -- arc deg  

			long i;
			double gr,r0,theta;
			double gain;
			double la_r,la_t;  //发射与接受天线

			if(ra_beam_ang<0.05)
			{
				ra_beam_ang=acos(href/(r_near+nr*dr))-acos(href/r_near);      
			}

			la_r=0.886/ra_beam_ang;
			la_t=0.886/ra_beam_ang;
			double sincv = 0.0;
	
			gr = r_near+nr/2*dr;
			double rc = sqrt(gr*gr+href*href);
			theta = acos(href/rc);

			for(i=0;i<nr;i++)
			{
				gr=r_near+i*dr;
				r0=sqrt(gr*gr+href*href);
				theta=acos(href/r0)-r_ang;
				sincv = sinc(sin(theta)*la_t);
				gain=pow(r_near/r0,3/2)*sincv*sincv;  
				ampl_coef[i]=gain; 
			}

		}
		
		void imgProc::ampl_corr(unsigned char *img_data,float *ampl_coef,__int64 img_na,__int64 img_nr)
		{//OK 2016/06/03
	
			// 需要注意数据的存储顺序；按img_nr顺序存储；1,...,img_nr;

			__int64 h,i;
			unsigned char *temp = new unsigned char[img_nr];
			for(h=0;h<img_na;h++)
			{	
				for(i=0;i<img_nr;i++)  
				{	
					temp[i] = unsigned char(img_data[i+h*img_nr]*ampl_coef[i]);
				}
				memcpy(img_data+h*img_nr,temp,sizeof(unsigned char)*img_nr);
			}
			delete [] temp;
		}
		
		void imgProc::geo_corr(unsigned char *img_data,float ra_res,float r_near,float href,__int64 img_na,__int64 img_nr)
		{//OK 20160603
	
			__int64 h,i;
			double  R0;
	
			//////////////////////////////////////////////////
			float *rcoef = new float[img_nr];
			unsigned char *temp=new unsigned char[img_nr];
			int  *ra = new int[img_nr];

			/////////geometry correct/////////
			if(href>1) 
			{	
				double  r_far   = r_near+img_nr*ra_res;
				double  gr_near = sqrt(r_near*r_near-href*href);
					 
				for(i=0;i<img_nr;i++) 
				{ 
				   R0 = sqrt((gr_near+i*ra_res)*(gr_near+i*ra_res)+href*href);
				   ra[i] = int((R0-r_near)/ra_res)%(img_nr-1);
				   rcoef[i] = (R0-r_near)/ra_res-float(ra[i]);	   
				}
		
				for(h=0;h<img_na;h++)
				{	
					for(i=0;i<img_nr;i++)  
					{	
						temp[i] = unsigned char(img_data[*(ra+i)+h*img_nr]*rcoef[i]+img_data[*(ra+i)+1+h*img_nr]*(1-rcoef[i]));
					}
					memcpy(img_data+h*img_nr,temp,sizeof(unsigned char)*img_nr);
				}
			} 

			delete [] temp;
			delete [] ra;
			delete [] rcoef;
    
		}

		void imgProc::smooth(unsigned char *img_data,__int64 img_na,__int64 img_nr)
		{//OK 20160603
	
			unsigned char *temp = new unsigned char[img_na*img_nr];
			memcpy(temp,img_data,sizeof(char)*img_na*img_nr);
			memset(img_data,0,sizeof(char)*img_na*img_nr);

			long k,j,m,n;
			for(k=3;k<img_na-3;k++)
				for(j=3;j<img_nr-3;j++)
				{
					for(m=0;m<3;m++)
					for(n=0;n<3;n++)
						img_data[k*img_nr+j] += temp[(k-1+m)*img_nr+j-1+n]/9;
				}
			delete [] temp; temp = NULL;
    
		}

		template <class T>
		inline double imgProc::max_val(T *val, long num, long *loc)
		{
			long    i;
			long   max_loc = 0;
			double  max_val0;
			if (num<0) return 0.0;
			max_val0 = val[0];
			for (i = 0; i<num; i++)
			{
				if (val[i]>max_val0)
				{
					max_val0 = val[i];
					max_loc = i;
				}
			}
			*loc = max_loc;
			return  max_val0;
		}



		// *********  公共函数输出 ************

		string commonFuncs::getName(UINT8* ar)
		{// modified by luo 2016/06/03

			string strName;
			char   chAr[64];			
	
			taskInfoPosition tskPs;
			tskPs.initial();

			sarImageInfoPosition imgInfoPs;
			imgInfoPs.initial();

			muInfoPosition ps;
			ps.initial();

			fmtConvCl  fmtConv(ar);
			int dbTaskCode=(int)fmtConv.getResult(tskPs.missionCodes);
			sprintf_s(chAr,"%04d",dbTaskCode);
			strName.append(chAr);
			strName.append("_");

			
			int itDate=(int)fmtConv.getResult(ps.date_year);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.date_month);

			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.date_day);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);


			strName.append("_");

			itDate=(int)fmtConv.getResult(ps.time_hour);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.time_minutes);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.time_second);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			strName.append("_");

			//条代号，固定值01;周期号：
			strName.append("01_");
			int dbstrpNum=(int)fmtConv.getResult(imgInfoPs.loop_num);
			sprintf_s(chAr,"%05d",dbstrpNum);
			strName.append(chAr);
			strName.append("_");

			//sar模式
			int sarModelNumTemp=getSarModel(ar);

			//分辨率
			UINT32 sarResIndex1=(UINT32)fmtConv.getResult(imgInfoPs.sarRes);
			int sarResIndex2=sarResIndex1>>25 & 0x07;

			//开机次数
			int powerOnTime=(int)fmtConv.getResult(tskPs.powerOnCnt);
			sprintf_s(chAr,"%03d",powerOnTime);
			strName.append(chAr);

			strName.append("_");
			////有无损压缩
			//strName.append("Y002_");

			//飞机批号、飞机号
			//strName.append("01");
			int dbPlaneNum=(int)fmtConv.getResult(tskPs.planeNum);
			sprintf_s(chAr,"%04d",dbPlaneNum);
			strName.append(chAr);
			strName.append("_");

			double transType=fmtConv.getResult(tskPs.transType);
			double tTypeU=transType;

			//时传经常出现数值很大的SB错误,所以弄个补丁
			if(tTypeU>2)
			{ 
				transType=1;
			}
			int rateCompress=(int)fmtConv.getResult(tskPs.imgCompRate);
			
			if(transType==0)
				{				  					
					strName.append("001");
					strName.append("_");
					
				}
				else
				{
					rateCompress=rateCompress>999?999:rateCompress;
					sprintf_s(chAr,"%03d",rateCompress);
					strName.append(chAr);
					strName.append("_");
				}

			
			strName.append("0703");

			UINT32 wkMd=(UINT32)fmtConv.getResult(imgInfoPs.work_mode);
				
			UINT32 work_mode=(wkMd>>16) & 0x0F;

			switch (sarResIndex2)
			{

			case 1:
				if((int)work_mode==0)
				{
					strName.append("11");//0.3米条带
				}else
				{
					strName.append("04");//0.3米聚书
				}
				break;
			case 2:
				strName.append("01");//0.5米条带
				break;
			case 3:
				strName.append("02");//1米条带
				break;
			case 4:            //3米 条带和sarGMTI都有三米模式
				if((int)work_mode==0)
				{
					strName.append("03");
				}
				else
				{
					strName.append("05");
				}
				break;
			case 5:          //5米
				strName.append("06");
				break;
			case 6:
				strName.append("07");
				break;
			case 7:
				strName.append("08");
				break;
			default :
				if((int)work_mode==4)
				{
					strName.append("09");
				}else
				{
					strName.append("10");
				}
				break;
			}

			return strName;
		}
		
		//生成不叠加点迹的图像名称
		string commonFuncs::getRawPicName(UINT8 * ar)
		{
            string strName;
			char   chAr[64];			
	
			taskInfoPosition tskPs;
			tskPs.initial();

			sarImageInfoPosition imgInfoPs;
			imgInfoPs.initial();

			muInfoPosition ps;
			ps.initial();

			fmtConvCl  fmtConv(ar);
			int dbTaskCode=(int)fmtConv.getResult(tskPs.missionCodes);
			sprintf_s(chAr,"%04d",dbTaskCode);
			strName.append(chAr);
			strName.append("_");

			
			int itDate=(int)fmtConv.getResult(ps.date_year);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.date_month);

			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.date_day);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);


			strName.append("_");

			itDate=(int)fmtConv.getResult(ps.time_hour);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.time_minutes);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.time_second);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			strName.append("_");

			//条代号，固定值01;周期号：
			strName.append("01_");
			int dbstrpNum=(int)fmtConv.getResult(imgInfoPs.loop_num);
			sprintf_s(chAr,"%05d",dbstrpNum);
			strName.append(chAr);
			strName.append("_");

			//sar模式
			int sarModelNumTemp=getSarModel(ar);

			//分辨率
			UINT32 sarResIndex1=(UINT32)fmtConv.getResult(imgInfoPs.sarRes);
			int sarResIndex2=sarResIndex1>>25 & 0x07;

			//开机次数
			int powerOnTime=(int)fmtConv.getResult(tskPs.powerOnCnt);
			sprintf_s(chAr,"%03d",powerOnTime);
			strName.append(chAr);

			strName.append("_");
			////有无损压缩
			//strName.append("Y002_");

			//飞机批号、飞机号
			//strName.append("01");
			int dbPlaneNum=(int)fmtConv.getResult(tskPs.planeNum);
			sprintf_s(chAr,"%04d",dbPlaneNum);
			strName.append(chAr);
			strName.append("_");

			double transType=fmtConv.getResult(tskPs.transType);
			double tTypeU=transType;
			//时传经常出现数值很大的SB错误,所以弄个补丁
			if(tTypeU>2)
			{ 
				transType=1;
			}

			int rateCompress=(int)fmtConv.getResult(tskPs.imgCompRate);

			if(transType==0)
				{				  					
					strName.append("001");
					strName.append("_");
					
				}
				else
				{
					rateCompress=rateCompress>999?999:rateCompress;
					sprintf_s(chAr,"%03d",rateCompress);
					strName.append(chAr);
					strName.append("_");
				}

			
			strName.append("0703");

			UINT32 wkMd=(UINT32)fmtConv.getResult(imgInfoPs.work_mode);
				
			UINT32 work_mode=(wkMd>>16) & 0x0F;

			switch (sarResIndex2)
			{

			case 1:
				if((int)work_mode==0)
				{
					strName.append("11");//0.3米条带
				}else
				{
					strName.append("04");//0.3米聚书
				}
				break;
			case 2:
				strName.append("01");//0.5米条带
				break;
			case 3:
				strName.append("02");//1米条带
				break;
			case 4:            //3米 条带和sarGMTI都有三米模式
				if((int)work_mode==0)
				{
					strName.append("03");
				}
				else
				{
					strName.append("05");
				}
				break;
			case 5:          //5米
				strName.append("06");
				break;
			case 6:
				strName.append("07");
				break;
			case 7:
				strName.append("08");
				break;
			default :
				if((int)work_mode==4)
				{
					strName.append("09");
				}else
				{
					strName.append("10");
				}
				break;
			}

			return strName;
		}
	    
		//生成点目标的文件名称，随着开机次数和时间在变化
		string commonFuncs::getDataName(UINT8 * ar)
		{
			string strName;
			char   chAr[64];			
	
			taskInfoPosition tskPs;
			tskPs.initial();

			sarImageInfoPosition imgInfoPs;
			imgInfoPs.initial();

			muInfoPosition ps;
			ps.initial();

			fmtConvCl  fmtConv(ar);
			int dbTaskCode=(int)fmtConv.getResult(tskPs.missionCodes);
			sprintf_s(chAr,"%04d",dbTaskCode);
			strName.append(chAr);
			strName.append("_");

			int itDate=(int)fmtConv.getResult(ps.date_year);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.date_month);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.date_day);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			strName.append("_");

			itDate=(int)fmtConv.getResult(ps.time_hour);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.time_minutes);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.time_second);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);
			strName.append("_");

			//条代号，固定值01;周期号：
			strName.append("01_");
			int dbstrpNum=(int)fmtConv.getResult(imgInfoPs.loop_num);
			sprintf_s(chAr,"%05d",dbstrpNum);
			strName.append(chAr);
			strName.append("_");

			//sar模式
			int sarModelNumTemp=getSarModel(ar);

			//分辨率
			UINT32 sarResIndex1=(UINT32)fmtConv.getResult(imgInfoPs.sarRes);
			int sarResIndex2=sarResIndex1>>25 & 0x07;

			//开机次数
			int powerOnTime=(int)fmtConv.getResult(tskPs.powerOnCnt);
			sprintf_s(chAr,"%03d",powerOnTime);
			strName.append(chAr);

			strName.append("_");
			////有无损压缩
			//strName.append("Y002_");

			//飞机批号、飞机号
			//strName.append("01");
			int dbPlaneNum=(int)fmtConv.getResult(tskPs.planeNum);
			sprintf_s(chAr,"%04d",dbPlaneNum);
			strName.append(chAr);
			strName.append("_");

			double transType=fmtConv.getResult(tskPs.transType);
			double tTypeU=transType;

			//时传经常出现数值很大的SB错误,所以弄个补丁
			if(abs(tTypeU)>2)
			{ 
				transType=1;
			}
			int rateCompress=(int)fmtConv.getResult(tskPs.imgCompRate);

			if(transType==0)
			{
				//GMTI
				strName.append("M00_");
			}
			else
			{
				strName.append("M01_");
			}
			
			strName.append("0703");

			//
			if(sarModelNumTemp==3)//sarGMTI
			{
				strName.append("03");
			}
			else if(sarModelNumTemp==4)
			{
				strName.append("00");
			}

			return strName;

		}
	    
		//生成点图像融合的图像名称
		string commonFuncs::getMarkPicName(UINT8 *ar)
		{
 
			string strName;
			char   chAr[64];			
	
			taskInfoPosition tskPs;
			tskPs.initial();

			sarImageInfoPosition imgInfoPs;
			imgInfoPs.initial();

			muInfoPosition ps;
			ps.initial();

			fmtConvCl  fmtConv(ar);
			int dbTaskCode=(int)fmtConv.getResult(tskPs.missionCodes);
			sprintf_s(chAr,"%04d",dbTaskCode);
			strName.append(chAr);
			strName.append("_");

			
			int itDate=(int)fmtConv.getResult(ps.date_year);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.date_month);

			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.date_day);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);


			strName.append("_");

			itDate=(int)fmtConv.getResult(ps.time_hour);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.time_minutes);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.time_second);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			strName.append("_");

			//条代号，固定值01;周期号：
			strName.append("01_");
			int dbstrpNum=(int)fmtConv.getResult(imgInfoPs.loop_num);
			sprintf_s(chAr,"%05d",dbstrpNum);
			strName.append(chAr);
			strName.append("_");

			//sar模式
			int sarModelNumTemp=getSarModel(ar);

			//分辨率
			UINT32 sarResIndex1=(UINT32)fmtConv.getResult(imgInfoPs.sarRes);
			int sarResIndex2=sarResIndex1>>25 & 0x07;

			//开机次数
			int powerOnTime=(int)fmtConv.getResult(tskPs.powerOnCnt)+900;
			sprintf_s(chAr,"%03d",powerOnTime);
			strName.append(chAr);

			strName.append("_");
			////有无损压缩
			//strName.append("Y002_");

			//飞机批号、飞机号
			//strName.append("01");
			int dbPlaneNum=(int)fmtConv.getResult(tskPs.planeNum);
			sprintf_s(chAr,"%04d",dbPlaneNum);
			strName.append(chAr);
			strName.append("_");

			double transType=fmtConv.getResult(tskPs.transType);
			double tTypeU=transType;
			//时传经常出现数值很大的SB错误,所以弄个补丁
			if(tTypeU>2)
			{ 
				transType=1;
			}
			int rateCompress=(int)fmtConv.getResult(tskPs.imgCompRate);

			

			if(transType==0)
				{				  					
					strName.append("001");
					strName.append("_");					
				}
				else
				{
					rateCompress=rateCompress>999?999:rateCompress;
					sprintf_s(chAr,"%03d",rateCompress);
					strName.append(chAr);
					strName.append("_");
				}
			
			strName.append("0703");

			UINT32 wkMd=(UINT32)fmtConv.getResult(imgInfoPs.work_mode);
				
			UINT32 work_mode=(wkMd>>16) & 0x0F;
			
			switch (sarResIndex2)
			{

			case 1:
				if((int)work_mode==0)
				{
					strName.append("11");//0.3米条带
				}else
				{
					strName.append("04");//0.3米聚书
				}
				break;
			case 2:
				strName.append("01");//0.5米条带
				break;
			case 3:
				strName.append("02");//1米条带
				break;
			case 4:            //3米 条带和sarGMTI都有三米模式
				if((int)work_mode==0)
				{
					strName.append("03");
				}
				else
				{
					strName.append("05");
				}
				break;
			case 5:          //5米
				strName.append("06");
				break;
			case 6:
				strName.append("07");
				break;
			case 7:
				strName.append("08");
				break;
			default :
				if((int)work_mode==4)
				{
					strName.append("09");
				}else
				{
					strName.append("10");
				}
				break;
			}

			return strName;
		}
	    
		// 生成点迹文件名称，一次开机产生一个
		string commonFuncs::getDataName_luo(UINT8 * ar)
		{// modified by Luo 2016/4/1 

			string strName;
			char   chAr[64];			
			
			taskInfoPosition tskPs;
			tskPs.initial();

			sarImageInfoPosition imgInfoPs;
			imgInfoPs.initial();

			muInfoPosition ps;
			ps.initial();

			fmtConvCl  fmtConv(ar);

			static int powerOnTimes_prev = -1;
			static string strName_dat;

			int powerOnTime=(int)fmtConv.getResult(tskPs.powerOnCnt);
			
			if((powerOnTime != powerOnTimes_prev) && (powerOnTime > powerOnTimes_prev))
			{
				int dbTaskCode=(int)fmtConv.getResult(tskPs.missionCodes);
				sprintf_s(chAr,"%04d",dbTaskCode);
				strName.append(chAr);
				strName.append("_");

				int itDate=(int)fmtConv.getResult(ps.date_year);
				sprintf_s(chAr,"%02d",itDate);
				strName.append(chAr);

				itDate=(int)fmtConv.getResult(ps.date_month);
				sprintf_s(chAr,"%02d",itDate);
				strName.append(chAr);

				itDate=(int)fmtConv.getResult(ps.date_day);
				sprintf_s(chAr,"%02d",itDate);
				strName.append(chAr);

				strName.append("_");

				itDate=(int)fmtConv.getResult(ps.time_hour);
				sprintf_s(chAr,"%02d",itDate);
				strName.append(chAr);

				itDate=(int)fmtConv.getResult(ps.time_minutes);
				sprintf_s(chAr,"%02d",itDate);
				strName.append(chAr);

				itDate=(int)fmtConv.getResult(ps.time_second);
				sprintf_s(chAr,"%02d",itDate);
				strName.append(chAr);
				strName.append("_");

				//条代号，固定值01;周期号：
				strName.append("01_");
				int dbstrpNum=(int)fmtConv.getResult(imgInfoPs.loop_num);
				sprintf_s(chAr,"%05d",dbstrpNum);
				strName.append(chAr);
				strName.append("_");

				//sar模式
				int sarModelNumTemp=getSarModel(ar);

				//分辨率
				UINT32 sarResIndex1=(UINT32)fmtConv.getResult(imgInfoPs.sarRes);
				int sarResIndex2=sarResIndex1>>25 & 0x07;

				//开机次数
				int powerOnTime1 =(int)fmtConv.getResult(tskPs.powerOnCnt);
				sprintf_s(chAr,"%03d",powerOnTime1);
				strName.append(chAr);

				strName.append("_");
				////有无损压缩
				//strName.append("Y002_");

				//飞机批号、飞机号
				//strName.append("01");
				int dbPlaneNum=(int)fmtConv.getResult(tskPs.planeNum);
				sprintf_s(chAr,"%04d",dbPlaneNum);
				strName.append(chAr);
				strName.append("_");

				double transType=fmtConv.getResult(tskPs.transType);
				double tTypeU=transType;

				//时传经常出现数值很大的SB错误,所以弄个补丁
				if(abs(tTypeU)>2)
				{ 
					transType=1;
				}
				int rateCompress=(int)fmtConv.getResult(tskPs.imgCompRate);

				if(transType==0)
				{
					//GMTI
					strName.append("M00_");
				}
				else
				{
					strName.append("M01_");
				}
			
				strName.append("0703");

				//
				if(sarModelNumTemp==3)//sarGMTI
				{
					strName.append("03");
				}
				else if(sarModelNumTemp==4)
				{
					strName.append("00");
				}

				// update strname
				powerOnTimes_prev = powerOnTime;

				strName_dat = strName; 

			}else
			{
				strName = strName_dat;
			}

			return strName;

		}

		string commonFuncs::getNameByLoopNum(UINT8* ar,int stripNum)
		{

			string strName;
			char   chAr[64];			
	
			taskInfoPosition tskPs;
			tskPs.initial();

			sarImageInfoPosition imgInfoPs;
			imgInfoPs.initial();

			muInfoPosition ps;
			ps.initial();

			fmtConvCl  fmtConv(ar);
			int dbTaskCode=(int)fmtConv.getResult(tskPs.missionCodes);
			sprintf_s(chAr,"%04d",dbTaskCode);
			strName.append(chAr);
			strName.append("_");

			
			int itDate=(int)fmtConv.getResult(ps.date_year);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.date_month);

			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.date_day);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);


			strName.append("_");

			itDate=(int)fmtConv.getResult(ps.time_hour);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.time_minutes);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.time_second);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			strName.append("_");

			//条代号，固定值01;周期号：
			strName.append("01_");
			sprintf_s(chAr,"%05d",stripNum);
			strName.append(chAr);
			strName.append("_");

			//sar模式
			int sarModelNumTemp=getSarModel(ar);

			//分辨率
			UINT32 sarResIndex1=(UINT32)fmtConv.getResult(imgInfoPs.sarRes);
			int sarResIndex2=sarResIndex1>>25 & 0x07;

			//开机次数
			int powerOnTime=(int)fmtConv.getResult(tskPs.powerOnCnt) + 900; // modify by zhang : 900, TODO check only outPutGMTIInfo() call this function
			sprintf_s(chAr,"%03d",powerOnTime);
			strName.append(chAr);

			strName.append("_");
			////有无损压缩
			//strName.append("Y002_");

			//飞机批号、飞机号
			//strName.append("01");
			int dbPlaneNum=(int)fmtConv.getResult(tskPs.planeNum);
			sprintf_s(chAr,"%04d",dbPlaneNum);
			strName.append(chAr);
			strName.append("_");

			if(sarModelNumTemp!=3)  // modify by zhang : SAR/GMTI mode number is 3?
			{
			//GMTI
			strName.append("MTI_");
			}
			else
			{
				/* ==== modify by zhang ==== */
				double transType = fmtConv.getResult(tskPs.transType);
				if(transType > 2)transType = 1;

				int rateCompress = (int)fmtConv.getResult(tskPs.imgCompRate);
			
				if(transType == 0){				  					
					strName.append("001_");
				}else{
					rateCompress = rateCompress > 999 ? 999 : rateCompress;
					sprintf_s(chAr, "%03d_", rateCompress);
					strName.append(chAr);
				}

#if 0
			//压缩比
			int rateCompress=(int)fmtConv.getResult(tskPs.imgCompRate);
			sprintf_s(chAr,"%03d",rateCompress);
			strName.append(chAr);
			strName.append("_");
			//strName.append("008_");
#endif
				/* ========================= */
			}
			
			strName.append("0703");

			UINT32 wkMd=(UINT32)fmtConv.getResult(imgInfoPs.work_mode);
				
			UINT32 work_mode=(wkMd>>16) & 0x0F;

			switch (sarResIndex2)
			{

			case 1:
				if((int)work_mode==0)
				{
					strName.append("11");//0.3米条带
				}else
				{
					strName.append("04");//0.3米聚书
				}
				break;
			case 2:
				strName.append("01");//0.5米条带
				break;
			case 3:
				strName.append("02");//1米条带
				break;
			case 4:            //3米 条带和sarGMTI都有三米模式
				if((int)work_mode==0)
				{
					strName.append("03");
				}
				else
				{
					strName.append("05");
				}
				break;
			case 5:          //5米
				strName.append("06");
				break;
			case 6:
				strName.append("07");
				break;
			case 7:
				strName.append("08");
				break;
			default :
				if((int)work_mode==4)
				{
					strName.append("09");
				}else
				{
					strName.append("10");
				}
				break;
			}

			return strName;
		}
		//生成不叠加点迹的图像名称
		string commonFuncs::getRawPicName(UINT8 * ar,int stripNum)
		{
            string strName;
			char   chAr[64];			
	
			taskInfoPosition tskPs;
			tskPs.initial();

			sarImageInfoPosition imgInfoPs;
			imgInfoPs.initial();

			muInfoPosition ps;
			ps.initial();

			fmtConvCl  fmtConv(ar);
			int dbTaskCode=(int)fmtConv.getResult(tskPs.missionCodes);
			sprintf_s(chAr,"%04d",dbTaskCode);
			strName.append(chAr);
			strName.append("_");

			
			int itDate=(int)fmtConv.getResult(ps.date_year);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.date_month);

			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.date_day);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);


			strName.append("_");

			itDate=(int)fmtConv.getResult(ps.time_hour);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.time_minutes);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.time_second);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			strName.append("_");

			//条代号，固定值01;周期号：
			strName.append("01_");
			sprintf_s(chAr,"%05d",stripNum);
			strName.append(chAr);
			strName.append("_");

			//sar模式
			int sarModelNumTemp=getSarModel(ar);

			//分辨率
			UINT32 sarResIndex1=(UINT32)fmtConv.getResult(imgInfoPs.sarRes);
			int sarResIndex2=sarResIndex1>>25 & 0x07;

			//开机次数
			int powerOnTime=(int)fmtConv.getResult(tskPs.powerOnCnt);
			sprintf_s(chAr,"%03d",powerOnTime);
			strName.append(chAr);

			strName.append("_");


			//飞机批号、飞机号
			int dbPlaneNum=(int)fmtConv.getResult(tskPs.planeNum);
			sprintf_s(chAr,"%04d",dbPlaneNum);
			strName.append(chAr);
			strName.append("_");

            double transType=fmtConv.getResult(tskPs.transType);
			double tTypeU=transType;
			
			//时传经常出现数值很大的SB错误,所以弄个补丁

			if(tTypeU>2)
			{ 
				transType=1;
			}

			int rateCompress=(int)fmtConv.getResult(tskPs.imgCompRate);

			if(transType==0)
			{				  					
				strName.append("001");
				strName.append("_");
					
			}
			else
			{
				rateCompress=rateCompress>999?999:rateCompress;
				sprintf_s(chAr,"%03d",rateCompress);
				strName.append(chAr);
				strName.append("_");
			}

			strName.append("0703");

			UINT32 wkMd=(UINT32)fmtConv.getResult(imgInfoPs.work_mode);
				
			UINT32 work_mode=(wkMd>>16) & 0x0F;

			switch (sarResIndex2)
			{

			case 1:
				if((int)work_mode==0)
				{
					strName.append("11");//0.3米条带
				}else
				{
					strName.append("04");//0.3米聚书
				}
				break;
			case 2:
				strName.append("01");//0.5米条带
				break;
			case 3:
				strName.append("02");//1米条带
				break;
			case 4:            //3米 条带和sarGMTI都有三米模式
				if((int)work_mode==0)
				{
					strName.append("03");
				}
				else
				{
					strName.append("05");
				}
				break;
			case 5:          //5米
				strName.append("06");
				break;
			case 6:
				strName.append("07");
				break;
			case 7:
				strName.append("08");
				break;
			default :
				if((int)work_mode==4)
				{
					strName.append("09");
				}else
				{
					strName.append("10");
				}
				break;
			}

			return strName;
		}
		
		string commonFuncs::getCombPicName(UINT8 * ar,int stripNum)
		{
			string strName;
			char   chAr[64];			
	
			taskInfoPosition tskPs;
			tskPs.initial();

			sarImageInfoPosition imgInfoPs;
			imgInfoPs.initial();

			muInfoPosition ps;
			ps.initial();

			fmtConvCl  fmtConv(ar);
			int dbTaskCode=(int)fmtConv.getResult(tskPs.missionCodes);
			sprintf_s(chAr,"%04d",dbTaskCode);
			strName.append(chAr);
			strName.append("_");

			
			int itDate=(int)fmtConv.getResult(ps.date_year);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.date_month);

			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.date_day);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);


			strName.append("_");

			itDate=(int)fmtConv.getResult(ps.time_hour);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.time_minutes);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.time_second);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			strName.append("_");

			//条代号，固定值01;周期号：
			strName.append("01_");
			sprintf_s(chAr,"%05d",stripNum);
			strName.append(chAr);
			strName.append("_");

			//sar模式
			int sarModelNumTemp=getSarModel(ar);

			//分辨率
			UINT32 sarResIndex1=(UINT32)fmtConv.getResult(imgInfoPs.sarRes);
			int sarResIndex2=sarResIndex1>>25 & 0x07;

			//开机次数
			int powerOnTime=(int)fmtConv.getResult(tskPs.powerOnCnt); // reove+800;
			sprintf_s(chAr,"%03d",powerOnTime);
			strName.append(chAr);

			strName.append("_");


			//飞机批号、飞机号
			int dbPlaneNum=(int)fmtConv.getResult(tskPs.planeNum);
			sprintf_s(chAr,"%04d",dbPlaneNum);
			strName.append(chAr);
			strName.append("_");

            double transType=fmtConv.getResult(tskPs.transType);
			double tTypeU=transType;
			//时传经常出现数值很大的SB错误,所以弄个补丁
			if(tTypeU>2)
			{ 
				transType=1;
			}

			int rateCompress=(int)fmtConv.getResult(tskPs.imgCompRate);

			if(transType==0)
			{				  					
				strName.append("001");
				strName.append("_");
					
			}
			else
			{
				rateCompress=rateCompress>999?999:rateCompress;
				sprintf_s(chAr,"%03d",rateCompress);
				strName.append(chAr);
				strName.append("_");
			}

			
			strName.append("0703");

			UINT32 wkMd=(UINT32)fmtConv.getResult(imgInfoPs.work_mode);
				
			UINT32 work_mode=(wkMd>>16) & 0x0F;

			switch (sarResIndex2)
			{

			case 1:
				if((int)work_mode==0)
				{
					strName.append("11");//0.3米条带
				}else
				{
					strName.append("04");//0.3米聚书
				}
				break;
			case 2:
				strName.append("01");//0.5米条带
				break;
			case 3:
				strName.append("02");//1米条带
				break;
			case 4:            //3米 条带和sarGMTI都有三米模式
				if((int)work_mode==0)
				{
					strName.append("03");
				}
				else
				{
					strName.append("05");
				}
				break;
			case 5:          //5米
				strName.append("06");
				break;
			case 6:
				strName.append("07");
				break;
			case 7:
				strName.append("08");
				break;
			default :
				if((int)work_mode==4)
				{
					strName.append("09");
				}else
				{
					strName.append("10");
				}
				break;
			}

			return strName;
		}
	    
		string commonFuncs::getDataName(UINT8 * ar,int stripNum)
		{
			string strName;
			char   chAr[64];			
	
			taskInfoPosition tskPs;
			tskPs.initial();

			sarImageInfoPosition imgInfoPs;
			imgInfoPs.initial();

			muInfoPosition ps;
			ps.initial();

			fmtConvCl  fmtConv(ar);
			int dbTaskCode=(int)fmtConv.getResult(tskPs.missionCodes);
			sprintf_s(chAr,"%04d",dbTaskCode);
			strName.append(chAr);
			strName.append("_");

			
			int itDate=(int)fmtConv.getResult(ps.date_year);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.date_month);

			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.date_day);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);


			strName.append("_");

			itDate=(int)fmtConv.getResult(ps.time_hour);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.time_minutes);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.time_second);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			strName.append("_");

			//条代号，固定值01;周期号：
			strName.append("01_");
			sprintf_s(chAr,"%05d",stripNum);
			strName.append(chAr);
			strName.append("_");

			//sar模式
			int sarModelNumTemp=getSarModel(ar);

			//分辨率
			UINT32 sarResIndex1=(UINT32)fmtConv.getResult(imgInfoPs.sarRes);
			int sarResIndex2=sarResIndex1>>25 & 0x07;

			//开机次数
			int powerOnTime=(int)fmtConv.getResult(tskPs.powerOnCnt);
			sprintf_s(chAr,"%03d",powerOnTime);
			strName.append(chAr);

			strName.append("_");

			//飞机批号、飞机号
			int dbPlaneNum=(int)fmtConv.getResult(tskPs.planeNum);
			sprintf_s(chAr,"%04d",dbPlaneNum);
			strName.append(chAr);
			strName.append("_");

			double transType=fmtConv.getResult(tskPs.transType);
			double tTypeU=transType;
			//时传经常出现数值很大的SB错误,所以弄个补丁
			if(tTypeU>2)
			{ 
				transType=1;
			}
			int rateCompress=(int)fmtConv.getResult(tskPs.imgCompRate);

			
			if(transType==0)
			{
				//GMTI
				strName.append("M00_");
			}
			else
			{
				strName.append("M01_");
			}
			
			strName.append("0703");

			//
			if(sarModelNumTemp==3)//sarGMTI
			{
				strName.append("03");
			}
			else if(sarModelNumTemp==4)
			{
				strName.append("00");
			}

			return strName;

		}
	    
		string commonFuncs::getMarkPicName(UINT8 *ar,int stripNum)
		{
 
			string strName;
			char   chAr[64];			
	
			taskInfoPosition tskPs;
			tskPs.initial();

			sarImageInfoPosition imgInfoPs;
			imgInfoPs.initial();

			muInfoPosition ps;
			ps.initial();

			fmtConvCl  fmtConv(ar);
			int dbTaskCode=(int)fmtConv.getResult(tskPs.missionCodes);
			sprintf_s(chAr,"%04d",dbTaskCode);
			strName.append(chAr);
			strName.append("_");

			
			int itDate=(int)fmtConv.getResult(ps.date_year);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.date_month);

			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.date_day);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);


			strName.append("_");

			itDate=(int)fmtConv.getResult(ps.time_hour);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.time_minutes);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			itDate=(int)fmtConv.getResult(ps.time_second);
			sprintf_s(chAr,"%02d",itDate);
			strName.append(chAr);

			strName.append("_");

			//条代号，固定值01;周期号：
			strName.append("01_");
			sprintf_s(chAr,"%05d",stripNum);
			strName.append(chAr);
			strName.append("_");

			//sar模式
			int sarModelNumTemp=getSarModel(ar);

			//分辨率
			UINT32 sarResIndex1=(UINT32)fmtConv.getResult(imgInfoPs.sarRes);
			int sarResIndex2=sarResIndex1>>25 & 0x07;

			//开机次数
			int powerOnTime=(int)fmtConv.getResult(tskPs.powerOnCnt)+900;
			sprintf_s(chAr,"%03d",powerOnTime);
			strName.append(chAr);

			strName.append("_");

			//飞机批号、飞机号
			int dbPlaneNum=(int)fmtConv.getResult(tskPs.planeNum);
			sprintf_s(chAr,"%04d",dbPlaneNum);
			strName.append(chAr);
			strName.append("_");

			double transType=fmtConv.getResult(tskPs.transType);
			double tTypeU=transType;
			//时传经常出现数值很大的SB错误,所以弄个补丁
			if(tTypeU>2)
			{ 
				transType=1;
			}
			int rateCompress=(int)fmtConv.getResult(tskPs.imgCompRate);

			if(transType==0)
				{				  					
					strName.append("001");
					strName.append("_");					
				}
				else
				{
					rateCompress=rateCompress>999?999:rateCompress;
					sprintf_s(chAr,"%03d",rateCompress);
					strName.append(chAr);
					strName.append("_");
				}
			
			strName.append("0703");

			UINT32 wkMd=(UINT32)fmtConv.getResult(imgInfoPs.work_mode);
				
			UINT32 work_mode=(wkMd>>16) & 0x0F;
			
			switch (sarResIndex2)
			{
			case 1:
				strName.append("04");//0.3米聚束
				break;
			case 2:
				strName.append("01");//0.5米条带
				break;
			case 3:
				strName.append("02");//1米条带
				break;
			case 4:            //3米 条带和sarGMTI都有三米模式
				if((int)work_mode==0)
				{
					strName.append("03");
				}
				else
				{
					strName.append("05");
				}
				break;
			case 5:          //5米
				strName.append("06");
				break;
			case 6:
				strName.append("07");
				break;
			case 7:
				strName.append("08");
				break;
			}

			return strName;
		}
		
		//以当前时间为标准
		string commonFuncs::getNameLocalTime(UINT8* ar,int stripNum)
		{
			string strName;
			char   chAr[64];			
	
			taskInfoPosition tskPs;
			tskPs.initial();

			fmtConvCl  fmtConv(ar);

			int dbTaskCode=(int)fmtConv.getResult(tskPs.missionCodes);
			sprintf_s(chAr,"%04d",dbTaskCode);
			strName.append(chAr);
			strName.append("_");

			//获取日期、时间
			time_t rawTime;
			struct tm * tmInfo;
			time(&rawTime);
			tmInfo=localtime(&rawTime);
			sprintf_s(chAr,"%02d",tmInfo->tm_mon+1);
			strName.append(chAr);
			sprintf_s(chAr,"%02d",tmInfo->tm_mday);
			strName.append(chAr);
			sprintf_s(chAr,"%02d",(tmInfo->tm_year-100));
			strName.append(chAr);
			strName.append("_");

			sprintf_s(chAr,"%02d",tmInfo->tm_hour);
			strName.append(chAr);
			sprintf_s(chAr,"%02d",tmInfo->tm_min);
			strName.append(chAr);
			sprintf_s(chAr,"%02d",tmInfo->tm_sec);
			strName.append(chAr);
			strName.append("_");

			//条代号，固定值01;周期号：
			strName.append("01_");
			sprintf_s(chAr,"%05d",stripNum);
			strName.append(chAr);
			strName.append("_");
			
			//有无损压缩
			strName.append("Y002_");
			//飞机批号、飞机号
			strName.append("01");
			int dbPlaneNum=(int)fmtConv.getResult(tskPs.planeNum);
			sprintf_s(chAr,"%04d",dbPlaneNum);
			strName.append(chAr);
			strName.append("_");
			//压缩比
			strName.append("08_");
			//
			strName.append("0703");
			//string str9(asctime(tmInfo));
			return strName;
		}
		
		//搜索到01dcef18,则返回帧头位置，否则返回-1
		int commonFuncs::getHeadPosition(UINT8 *dataAr,int arLen)
		{

			long k;
			for(k=0; k<arLen-4; k++)
			{
				if(dataAr[k] == 0x01 && dataAr[k+1] == 0xDC && dataAr[k+2] == 0xEF && dataAr[k+3] == 0x18)
				{
					break;
				}
			}

			if(k >= arLen-4) 
				return -1;
			else  
				return k;

			/*
			const int *dtahead=DT_HEAD;
			int headLen=HD_LEN;
			int cntAr=0;
			int cntHd=0;
			while(cntAr<arLen&&cntHd<headLen)
			{
				
				if(dataAr[cntAr]==dtahead[cntHd])
				{
					cntAr++;
					cntHd++;
				}
				else
				{
					cntAr=cntAr-cntHd+1;
					cntHd=0;
				}
			}

			if(cntHd==headLen)
			{
				return (cntAr-headLen);
			}
			else
			{
				return -1;
			}
			*/
  
		}

		__int64 commonFuncs::getHeadPosition_i64(UINT8 *dataAr,__int64 arLen)
		{

			const int *dtahead=DT_HEAD;
			int headLen=HD_LEN;
			__int64 cntAr=0;
			__int64 cntHd=0;
			while(cntAr<arLen&&cntHd<headLen)
			{
				
				if(dataAr[cntAr]==dtahead[cntHd])
				{
					cntAr++;
					cntHd++;
				}
				else
				{
					cntAr=cntAr-cntHd+1;
					cntHd=0;
				}
			}

			if(cntHd==headLen)
			{
				return (cntAr-headLen);
			}
			else
			{
				return -1;
			}
  
		}

		//搜索到55AA55AA,则返回帧头位置，否则返回-1
		int commonFuncs::getHead6SuoDatePosition(UINT8 *dataAr,int arLen)
		{

			const int *dtahead=DT6Suo_HEAD;
			int headLen=HD_LEN;
			int cntAr=0;
			int cntHd=0;
			while(cntAr<arLen&&cntHd<headLen)
			{
				
				if(dataAr[cntAr]==dtahead[cntHd])
				{
					cntAr++;
					cntHd++;
				}
				else
				{
					cntAr=cntAr-cntHd+1;
					cntHd=0;
				}
			}

			if(cntHd==headLen)
			{
				return (cntAr-headLen);
			}
			else
			{
				return -1;
			}
  
		}
		
		int commonFuncs::getHead6SuoDateFrameLen(UINT8 *dataAr)
		{
			int mdPs=2*0x04;
			fmtConvCl fmtCnv(dataAr);		
			UINT32 md=fmtCnv.byteToUint32(mdPs);			
			return	md; 
		}

		int commonFuncs::getHeadPosition(UINT8 *dataAr,int arLen,int *rePos)
		{
			const int *dtahead=DT_HEAD;
			int headLen=HD_LEN;
			int cntHd=0;
			int cntAr=0;
			int posLen=0;
			while(cntAr<arLen)
			{
				if(dataAr[cntAr]==dtahead[cntHd])
				{										
					if(cntHd==(headLen-1))
					{
						posLen++;
						rePos[posLen-1]=(cntAr-cntHd);	
						//debug info
						//logRecords("frame position:",posLen-1);
						//调整帧头计数
						cntHd=-1;
					}
					cntAr++;
					cntHd++;
				}
				else
				{
					cntAr=cntAr-cntHd+1;
					cntHd=0;
				}

			}

				return posLen;
			
		}

		int commonFuncs::getSarModel(UINT8 * ar)
		{
			
			int mdPs=4*0x04;
			fmtConvCl fmtCnv(ar);
			UINT32 mdTemp=fmtCnv.byteToUint32(mdPs);
			int md=(mdTemp>>16)&0x0F;
			
			return	md;

		}

		int commonFuncs::getSarRes(UINT8 *ar)
		{
			//分辨率
            int mdPs=4*0x0F;
			fmtConvCl fmtCnv(ar);
		
			UINT32 mdTemp=fmtCnv.byteToUint32(mdPs);
			int md=(mdTemp>>25) & 0x07;
			
			return	md;
		}

		int commonFuncs::getRangePoints(UINT8 *ar)
		{
			stParInFrame ptFr;
			ptFr.startPosion=510;
			ptFr.numType=uInt16Enm;
			ptFr.unit=512;

			fmtConvCl fmtConv(ar);
			return  (int)fmtConv.getResult(ptFr);

		}

		int commonFuncs::searchHeadPosition(FILE * FL)
		{
			int hdLen=512;//帧头有512个字节
			UINT32 step=513*100;

			UINT8 *searchBuffer=new UINT8[step];
			UINT8 *headerAr=new UINT8[hdLen];
			int endMark=feof(FL);

			while(endMark==0)
			{
			  UINT32 rdState=fread(searchBuffer,sizeof(UINT8),step,FL);

			  if(rdState<hdLen)
			  {
				  endMark=feof(FL);
				  break;
			  }

			  int re=getHeadPosition(searchBuffer,rdState);
			  
			  if(re!=-1)
			  {
				 
				 fseek(FL,re-rdState,SEEK_CUR);
				 break;
			  }	 
			  else 
			  {				 
					//防止帧头标志位处于两帧分割之间
					fseek(FL,-1*HD_LEN,SEEK_CUR);
			  }	

			}

			delete [] searchBuffer;
			delete [] headerAr;

			return endMark; 
		}

		int commonFuncs::searchHeadPositionX64(FILE * FL)
		{

			int hdLen=512;//帧头有512个字节
			UINT32 step=512*100;

			UINT8 *searchBuffer=new UINT8[step];
			UINT8 *headerAr=new UINT8[hdLen];
			int endMark=feof(FL);

			while(endMark==0)
			{
			  UINT32 rdState=fread(searchBuffer,sizeof(UINT8),step,FL);
			  int re=getHeadPosition(searchBuffer,rdState);
			  
			  if(re!=-1)
			  {
				/* long ll1=ftell(FL);
				 long gg1=_ftelli64(FL);*/
                 long backDis=re-rdState;
				 //fseek(FL,re-rdState,SEEK_CUR);
				 int pp=_fseeki64(FL,backDis,SEEK_CUR);
				 /*long ll=ftell(FL);
				 long gg=_ftelli64(FL);*/
				 break;
			  }	 

			   endMark=feof(FL);
			}

			delete [] searchBuffer;
			delete [] headerAr;

			return endMark;
		}

		void commonFuncs::logRecords(char* chMessage,double cnt)
		{// modified by Yunhua-Luo is OK 2021/01/18

			double temp = double(cnt);
			if(recordsPrint==true)
			{
			   FILE * FL;
			   FL = fopen("log.txt","at+");
			   fprintf(FL,"%s %.2f\n",chMessage,temp);			   
			   fclose(FL);
			}	
		}
		
		template <class T>
		void commonFuncs::logRecords_mt(char* chMessage,T cnt)
		{// modified by Yunhua-Luo is OK 2016/1/5

			FILE * FL;
			double temp = double(cnt);
			if(recordsPrint==true)
			{
			   FILE * FL;
			   if((FL=fopen("log_mt.txt","a"))==NULL) return;
			   fprintf(FL,"%s %.2f\n",chMessage,temp);		   
			   fclose(FL);
			}
		}

		UINT64 commonFuncs::getPRFNum(UINT8 * ar)
		{
			stParInFrame prfNumPs;
			prfNumPs.startPosion=4;
			prfNumPs.numType=uInt32Enm;
			prfNumPs.unit=1;

			fmtConvCl fmtConv(ar);
			return  (UINT64)fmtConv.getResult(prfNumPs);
		}

		int commonFuncs::getNewGYAimsNum(UINT8 *ar)
		{
				NewGYAimInfoPosition ps;
				ps.initial();
				fmtConvCl fmtConv(ar);			
				int asNum=fmtConv.getBigEndianResult(ps.aimsNum);
				return asNum;
		}

		int commonFuncs::getMViewNum(UINT8 *ar)
		{// updated 2016/7/22

			int multilook = 2;
			multilook = ar[0x72*4+1];  // 2016/7 后的非检飞版本新版直接读取多视数
			int md = getSarModel(ar);  // 2016/7 对于SAR-GMTI直接判断；
			if(md == 3) multilook = 4;
			if(multilook == 0) multilook = 2;
			
			return  multilook;
		}

		int commonFuncs::getPowerOnTime(UINT8 * ar)
		{
			taskInfo tskInfo(ar);

			return (int)tskInfo.powerOnCnt;	
		}

		int commonFuncs::getLoopNum(UINT8 *ar) 
		{
			unsigned char loopnum = ar[0x72*4];

			return loopnum;
		}
		
		double commonFuncs::getDopplerCenter(UINT8 *ar, double prf) 
		{
			long temp = 0;
			double fdc = 0;
			temp = ar[0x70*4+3]*pow(2.0,24)+ar[0x70*4+2]*pow(2.0,16)+ar[0x70*4+1]*pow(2.0,8)+ar[0x70*4];

			/*
			float *pfloat;
			pfloat = (float *)(&ar[0x70 * 4]);
			fdc = pfloat[0];
			*/
			
			if(temp < 32768) 
				fdc = double(temp)/65536*prf;
			else
				fdc = double(temp)/65536*prf-prf;
			
			return fdc;
		}

		string commonFuncs::getSysTime(FILE *fl)
		{
			string strName;

			char chAr[64];
			
			//获取日期、时间
			time_t rawTime;
			struct tm * tmInfo;
			time(&rawTime);
			tmInfo=localtime(&rawTime);
						
			sprintf_s(chAr,"%02d",(tmInfo->tm_year-100));
			strName.append(chAr);
			sprintf_s(chAr,"%02d",tmInfo->tm_mon+1);
			strName.append(chAr);
			sprintf_s(chAr,"%02d",tmInfo->tm_mday);
			strName.append(chAr);
			strName.append("  ");
			sprintf_s(chAr,"%02d",tmInfo->tm_hour);
			strName.append(chAr);
			strName.append(":");
			sprintf_s(chAr,"%02d",tmInfo->tm_min);
			strName.append(chAr);
			strName.append(":");
			sprintf_s(chAr,"%02d",tmInfo->tm_sec);
			strName.append(chAr);

			if(fl!=NULL)
			{				
              fprintf(fl,"系统时间:%s\n",strName.c_str());
			}

			return strName;
		}
		
		int commonFuncs::getWasMTIBagNo(UINT8 *dataAr)
		{
			if(dataAr == NULL) return 1;

			return dataAr[517]; // 分包号 , dataAr[518]为总包数

		}

		int commonFuncs::getWasMTIBagNum(UINT8 *dataAr)
		{
			if(dataAr == NULL) return 1;

			return dataAr[518]; // 分包号 , dataAr[518]为总包数
		}

		void commonFuncs::OutputRDLocatingInfo(FILE *FL, sarImageInfo *ps, sarImageInfo *pe)
		{

			fprintf(FL, "\n\********* 地理编码和几何校正参数  **********\n");

			fprintf(FL, "飞机起始经度(度):%6.7f\n", ps->plane_longitude);
			fprintf(FL, "飞机起始纬度(度):%6.7f\n", ps->plane_latitude);
			fprintf(FL, "飞机结束经度(度):%6.7f\n", pe->plane_longitude);
			fprintf(FL, "飞机结束纬度(度):%6.7f\n", pe->plane_latitude);
			fprintf(FL, "飞机海拔高度(米):%.3f\n", pe->plane_height);
			fprintf(FL, "成像场景海拔高度(米):%.3f\n", pe->aim_height);

			fprintf(FL, "飞机起始东速(米/秒):%.4f\n", ps->plane_east_v);
			fprintf(FL, "飞机起始北速(米/秒):%.4f\n", ps->plane_north_v);
			fprintf(FL, "飞机起始天速(米/秒):%.4f\n", ps->plane_up_v);

			fprintf(FL, "飞机结束东速(米/秒):%.4f\n", pe->plane_east_v);
			fprintf(FL, "飞机结束北速(米/秒):%.4f\n", pe->plane_north_v);
			fprintf(FL, "飞机结束天速(米/秒):%.4f\n", pe->plane_up_v);

			if (ps->rsAngle>0)
			{
				fprintf(FL, "侧视方式(0=左侧视,1=右侧视):0\n");  // modify by zhang
			}
			else
			{
				fprintf(FL, "侧视方式(0=左侧视,1=右侧视):1\n");  // modify by zhang
			}

			if (ps->ml2_coef < 1)
			{
				fprintf(FL, "方位向像元尺寸(米):%02.3f\n", ps->a_point_size*ps->multipleView / ps->ml2_coef);
				fprintf(FL, "距离向像元尺寸(米):%02.3f\n", ps->r_point_size);
			}
			else
			{
				fprintf(FL, "方位向像元尺寸(米):%02.3f\n", ps->a_point_size*ps->multipleView);
				fprintf(FL, "距离向像元尺寸(米):%02.3f\n", ps->r_point_size*ps->ml2_coef);
			}

			fprintf(FL, "多普勒中心频率(HZ):%.2f\n", ps->dopler_center);
			fprintf(FL, "雷达波长(米): 0.03124\n");
			fprintf(FL, "最近斜距(米):%.2f\n", ps->slope_nearest);
			fprintf(FL, "最远斜距(米):%.2f\n", ps->slope_far);

			fprintf(FL, "\n*************************\n");

		}

	// DATA TO TXT FUNCTIONS 

		void commonFuncs::ReadXL_EMapFile_mode_1(char* filename,char* outputdir)
{
		
	 //1.读取导引头位置信息
		 FILE *fid = NULL;
		 FILE *fidtxt = NULL;

		 __int64 M,N,offset,len,kk,jj;
		 double lati,logn;
		 int track_num, tar_num;
		 int state = 0;
		 
		 fid = fopen(filename,"rb");
		 if(fid==NULL)
		 {
			 printf("Cannot open this file\n");
			 return;
		  }
		  fseek(fid,0,SEEK_END);
		  M = ftell(fid);
		  fseek(fid,0,SEEK_SET);

		  unsigned char* info = new unsigned char[M];
		  fread(info,sizeof(unsigned char),M,fid);
		  fclose(fid);
		 
		  long*  pos = new long[2000000];
		  int cnt = 0;
		  long prev_no = -1;
		  for(kk=0;kk<M-6;kk++)
		  {
			  if(info[kk] == 170 && info[kk+1] == 85 && info[kk+2] == 170 && info[kk+3]==85)
			  {
				  if(cnt >199999) break;
				  pos[cnt] = kk;
				  cnt++;
			  }
		  }
		  delete [] info;
	
		  //Read target information
		  fid = fopen(filename,"rb");
		  if(fid==NULL)
		  {
			 printf("Cannot open filename\n");
			 return;
		  }
		  printf("============ MTI mode moving targets track output ============\n");
		  printf("%s\n",filename);
		  N = cnt;
		  char txtfile[512];
		  if(cnt == 0) return;

		  for(kk=0;kk<N;kk++)
		  {
			  offset = pos[kk];
              fseek(fid,offset,SEEK_SET);
		      UINT32 len[3];
              fread(len,sizeof(UINT32),3,fid);
              UINT32 mode = len[1];
			  
			  if(mode == 2) // MTI航迹
			  {
				  track_num = int((len[2]-216)/36);
			  }

			  if(mode == 1) // MTI点迹
			  {
				  tar_num = int((len[2]-216)/28);
			  }

			  if(mode == 3) // SAR/GMTI点迹
			  {
				  tar_num = int((len[2]-216)/40);
			  }

			  if(mode <= 0 || mode >3) return;
		   
		   //read information
		    UINT16 taskcode;
			fread(&taskcode,sizeof(UINT16),1,fid);
			UINT8 planeType;
			fread(&planeType,sizeof(UINT8),1,fid);
			UINT8 planeNum;
			fread(&planeNum,sizeof(UINT8),1,fid);
			UINT16 planenum;
			fread(&planenum,sizeof(UINT16),1,fid);
			UINT8 pic_compresee_ratio;
			fread(&pic_compresee_ratio,sizeof(UINT8),1,fid);
			UINT8 transType;
			fread(&transType,sizeof(UINT8),1,fid);
			UINT8 loadType;
			fread(&loadType,sizeof(UINT8),1,fid);
			UINT32 loadNum;
			fread(&loadNum,sizeof(UINT32),1,fid);
			UINT16 openTimes;
			fread(&openTimes,sizeof(UINT16),1,fid);
			UINT32 IMUnum;
			fread(&IMUnum,sizeof(UINT32),1,fid);

		    UINT16 year;
			fread(&year,sizeof(UINT16),1,fid);
			UINT8 month;
			fread(&month,sizeof(UINT8),1,fid);
			UINT8 day;
			fread(&day,sizeof(UINT8),1,fid);
			UINT8 hour;
			fread(&hour,sizeof(UINT8),1,fid);
			UINT8 min;
			fread(&min,sizeof(UINT8),1,fid);
			UINT8 sec;
			fread(&sec,sizeof(UINT8),1,fid);
			UINT16 msec;
			fread(&msec,sizeof(UINT16),1,fid);

			double plane_logn;
			fread(&plane_logn,sizeof(double),1,fid);
			double plane_lati;
			fread(&plane_lati,sizeof(double),1,fid);

			float plane_height;
			fread(&plane_height,sizeof(float),1,fid);
			float plane_point_height;
			fread(&plane_point_height,sizeof(float),1,fid);
			
			float plane_track_angle;
			fread(&plane_track_angle,sizeof(float),1,fid);
			float plane_track_angle_v;
			fread(&plane_track_angle_v,sizeof(float),1,fid);
			float plane_track_angle_a;
			fread(&plane_track_angle_a,sizeof(float),1,fid);

			float plane_pitch_angle;
			fread(&plane_pitch_angle,sizeof(float),1,fid);
			float plane_pitch_angle_v;
			fread(&plane_pitch_angle_v,sizeof(float),1,fid);
			float plane_pitch_angle_a;
			fread(&plane_pitch_angle_a,sizeof(float),1,fid);

			float plane_roll_angle;
			fread(&plane_roll_angle,sizeof(float),1,fid);
			float plane_roll_angle_v;
			fread(&plane_roll_angle_v,sizeof(float),1,fid);
			float plane_roll_angle_a;
			fread(&plane_roll_angle_a,sizeof(float),1,fid);

			float plane_drift_angle;
			fread(&plane_drift_angle,sizeof(float),1,fid);
			float plane_crab_angle;
			fread(&plane_crab_angle,sizeof(float),1,fid);

			float ground_v;
			fread(&ground_v,sizeof(float),1,fid);
			float true_airspeed;
			fread(&true_airspeed,sizeof(float),1,fid);
			float indicator_airspeed;
			fread(&indicator_airspeed,sizeof(float),1,fid);
			float planespeed_east;
			fread(&planespeed_east,sizeof(float),1,fid);
			float planespeed_north;
			fread(&planespeed_north,sizeof(float),1,fid);
			float planespeed_sky;
			fread(&planespeed_sky,sizeof(float),1,fid);
			float plane_a_speed_east;
			fread(&plane_a_speed_east,sizeof(float),1,fid);
			float plane_a_speed_north;
			fread(&plane_a_speed_north,sizeof(float),1,fid);
			float plane_a_speed_sky;
			fread(&plane_a_speed_sky,sizeof(float),1,fid);

			UINT32 frameno;
			fread(&frameno,sizeof(UINT32),1,fid);
			UINT16 wavenum;
			fread(&wavenum,sizeof(UINT16),1,fid);
			UINT16 waveno;
			fread(&waveno,sizeof(UINT16),1,fid);

			UINT8 freq;
			fread(&freq,sizeof(UINT8),1,fid);
			UINT8 sideway;
			fread(&sideway,sizeof(UINT8),1,fid);
			UINT8 workmode;
			fread(&workmode,sizeof(UINT8),1,fid);
			UINT8 worksubmode;
			fread(&worksubmode,sizeof(UINT8),1,fid);

			UINT32 r_far;
			fread(&r_far,sizeof(UINT32),1,fid);
			UINT32 r_near;
			fread(&r_near,sizeof(UINT32),1,fid);

			float ScanCenterAng;
			fread(&ScanCenterAng,sizeof(float),1,fid);
			float ScanScope;
			fread(&ScanScope,sizeof(float),1,fid);
			float azimuthCenterAngle;
			fread(&azimuthCenterAngle,sizeof(float),1,fid);
			float beam_horz_width;
			fread(&beam_horz_width,sizeof(float),1,fid);
			float beam_scan_step;
			fread(&beam_scan_step,sizeof(float),1,fid);
			float lookangle;
			fread(&lookangle,sizeof(float),1,fid);
			float beam_R_width;
			fread(&beam_R_width,sizeof(float),1,fid);
			float beam_R_step;
			fread(&beam_R_step,sizeof(float),1,fid);

			UINT16 pulseResident;
			fread(&pulseResident,sizeof(UINT16),1,fid);
			UINT16 resTime;
			fread(&resTime,sizeof(UINT16),1,fid);
			float resolution;
			fread(&resolution,sizeof(float),1,fid);
			UINT32 prf;
			fread(&prf,sizeof(UINT32),1,fid);

			if(state == 0)
			{	
				if(fidtxt != NULL){fidtxt = NULL;}
				memset(txtfile,0,sizeof(char)*512);
				
				long year_s = year;
				if(year > 2000) year_s = year - 2000;

				sprintf(txtfile,"%s%04d_%02d%02d%02d_%02d%02d%02d_01_%05d_%03d_%04d_M%02d_%02d%02d%02d.txt",outputdir,taskcode,year_s,month,day,hour,min,sec,
				frameno,openTimes,planenum,mode,planeType,loadType,workmode);
				fidtxt = fopen(txtfile,"a");
				if(fidtxt == NULL){printf("open txtfile fail !\n");return;}
				state = state + 1;
			}
			
			fprintf(fidtxt,"\n\n信息标签:  %02d\n",mode);
			fprintf(fidtxt,"任务代号:  %04d\n",taskcode);
            fprintf(fidtxt,"飞行器类型:  %02d\n",planeType);
            fprintf(fidtxt,"飞机批号:  %02d\n",planeNum);
            fprintf(fidtxt,"飞机号:  %04d\n",planenum);
            fprintf(fidtxt,"图像压缩比:  %02d\n",pic_compresee_ratio);
            fprintf(fidtxt,"传输方式(0=地面回读,1=实时传输,2=选择传输,3=精细成像):  %d\n",transType);
            fprintf(fidtxt,"载荷类型(01-高空CCD, 02-长焦倾斜CCD，03-SAR，04-红外行扫仪，05-多光谱):  %02d\n",loadType);
            fprintf(fidtxt,"载荷编号:  %06d\n",loadNum);
            fprintf(fidtxt,"开机次数:  %03d\n",openTimes);
            fprintf(fidtxt,"惯导序号:  %05d\n",IMUnum);

			long year_s = year;
			if(year < 2000) year_s = year + 2000;
            fprintf(fidtxt,"日期:  %04d.%02d.%02d\n",year_s,month,day);
            fprintf(fidtxt,"时间:  %02d-%02d-%02d-%03d\n",hour,min,sec,msec);
            fprintf(fidtxt,"飞机经度(度):  %.7f\n",plane_logn);
            fprintf(fidtxt,"飞机纬度(度):  %.7f\n",plane_lati);
            fprintf(fidtxt,"飞机海拔高度(米):  %.3f\n",plane_height);
            fprintf(fidtxt,"机下点海拔高度(米):  %.3f\n",plane_point_height);
            fprintf(fidtxt,"飞机航向角(度):  %.7f\n",plane_track_angle);
            fprintf(fidtxt,"飞机航向角速度(度/秒):  %.3f\n",plane_track_angle_v);
            fprintf(fidtxt,"飞机航向角加速度(度/平方秒):  %.3f\n",plane_track_angle_a);
            fprintf(fidtxt,"飞机俯仰向角(度)  %.7f\n",plane_pitch_angle);
            fprintf(fidtxt,"飞机俯仰向角速度(度/秒): 0.000\n");
            fprintf(fidtxt,"飞机俯仰向角加速度(度/平方秒):  0.000\n");
            fprintf(fidtxt,"飞机横滚角(度):  %.7f\n",plane_roll_angle);
            fprintf(fidtxt,"飞机横滚角速度(度/秒):  0.000\n");
            fprintf(fidtxt,"飞机横滚角加速度(度/平方秒):  0.000\n");
            fprintf(fidtxt,"飞机偏航角:  %.7f\n",plane_drift_angle );
            fprintf(fidtxt,"飞机偏流角(度):  0.0000000\n");
            fprintf(fidtxt,"地速(米/秒):  %.4f\n",ground_v );
            fprintf(fidtxt,"真空速(米/秒):  0.0000\n");
            fprintf(fidtxt,"指示空速(米/秒):  0.0000\n"); 
            fprintf(fidtxt,"飞机东速(米/秒):  %.4f\n",planespeed_east );
            fprintf(fidtxt,"飞机北速(米/秒):  %.4f\n",planespeed_north );
            fprintf(fidtxt,"飞机天速(米/秒):  %.4f\n",planespeed_sky);  
            fprintf(fidtxt,"东向加速度(米/秒平方):  %.4f\n",plane_a_speed_east );
            fprintf(fidtxt,"北向加速度(米/秒平方):  %.4f\n",plane_a_speed_north);
            fprintf(fidtxt,"天向加速度(米/秒平方):  %.4f\n",plane_a_speed_sky);
            fprintf(fidtxt,"天线帧编号:  %06d\n",frameno);
            fprintf(fidtxt,"波位数(个):  %04d\n",wavenum);
            fprintf(fidtxt,"波位号:  %04d\n",waveno);
            fprintf(fidtxt,"工作频段(0=X，1=L，2=P，3=P+L，4=Ku):  %02d\n",freq);
            fprintf(fidtxt,"侧视方式(0=左侧视，1=右侧视):  %02d\n",sideway);
            fprintf(fidtxt,"工作模式(0=GMTI，1=MMTI，2=AMTI，3=SAR/MTI同时模式):  %02d\n",workmode);
            fprintf(fidtxt,"工作子模式(0=广域，1=扇区，2=跟踪，9=空缺): %02d\n",worksubmode);
            fprintf(fidtxt,"最大作用距离(米):  %d\n",r_far);
            fprintf(fidtxt,"最小作用距离(米):  %d\n",r_near);
 
            fprintf(fidtxt,"天线帧扫描中心角(度):  %.4f\n",ScanCenterAng);
            fprintf(fidtxt,"天线帧扫描范围(度):  %.4f\n",ScanScope);
            fprintf(fidtxt,"方位波束中心角(度):  %.4f\n",azimuthCenterAngle);
            fprintf(fidtxt,"方位波束宽度(度):  %.4f\n",beam_horz_width);
            fprintf(fidtxt,"方位向扫描步进(度):  %.4f\n",beam_scan_step);
            fprintf(fidtxt,"俯仰波束中心角(度):  %.4f\n",lookangle);
            fprintf(fidtxt,"俯仰波束宽度(度):  %.4f\n",beam_R_width);
            fprintf(fidtxt,"俯仰向扫描步进(度):  %.4f\n",beam_R_step);
            fprintf(fidtxt,"驻留脉冲数(个):  %04d\n",pulseResident);
            fprintf(fidtxt,"波束驻留时间(毫秒):  %05d\n",resTime);
            fprintf(fidtxt,"分辨率(米):  %.1f\n",resolution);
            fprintf(fidtxt,"重复频率(Hz):  %05d\n",prf);  

			//写航迹文件
			if(mode==2)  //output track information 
			{
				fprintf(fidtxt,"WAS-MTI航迹个数(个):  %05d\n",track_num);  
				for (jj = 0; jj<track_num; jj++)
				{
					fseek(fid,offset+212+36*jj,SEEK_SET); 
					UINT32 track_no;
					fread(&track_no,sizeof(UINT32),1,fid);
					double tar_gps[2];
					fread(tar_gps,sizeof(double),2,fid);
					float tar_high;
					fread(&tar_high,sizeof(float),1,fid);
					float tar_vr;
					fread(&tar_vr,sizeof(float),1,fid);
					float tar_dir;
					fread(&tar_dir,sizeof(float),1,fid);

					fprintf(fidtxt,"\n航迹批号: %04d\n",track_no);
                    fprintf(fidtxt,"目标经度(度): %.7f\n",tar_gps[0]);
                    fprintf(fidtxt,"目标纬度(度): %.7f\n",tar_gps[1]);
                    fprintf(fidtxt,"目标高度(米): %.3f\n",tar_high);
                    fprintf(fidtxt,"目标速度(米/秒): %.3f\n",tar_vr);
                    fprintf(fidtxt,"目标方向(度): %.4f\n",tar_dir);
                    fprintf(fidtxt,"新批标识(1=是,0=否): 1\n");
                    fprintf(fidtxt,"目标属性(0=敌,1=我,2=不明): 2\n");
				}
			}

			//写点迹文件
			if(mode==1)  //output track information 
			{
				fprintf(fidtxt,"WAS-MTI点迹个数(个):  %05d\n",tar_num);  
				for (jj = 0; jj<tar_num; jj++)
				{
					fseek(fid,offset+212+28*jj,SEEK_SET); 
					UINT32 point_no;
					fread(&point_no,sizeof(UINT32),1,fid);
					double tar_logn;
					fread(&tar_logn,sizeof(double),1,fid);
					double tar_lati;
					fread(&tar_lati,sizeof(double),1,fid);
					float tar_h;
					fread(&tar_h,sizeof(float),1,fid);
					float tar_vr;
					fread(&tar_vr,sizeof(float),1,fid);

					fprintf(fidtxt,"\n点迹编号: %04d\n",point_no);
                    fprintf(fidtxt,"点迹经度(度): %.7f\n",tar_logn);
                    fprintf(fidtxt,"点迹纬度(度): %.7f\n",tar_lati);
                    fprintf(fidtxt,"点迹高度(米): %.3f\n",tar_h);
                    fprintf(fidtxt,"径向速度(米/秒): %.3f\n",tar_vr);
				}
			}

			//写点迹文件
			if(mode==3)  //output track information 
			{
				fprintf(fidtxt,"SAR-GMTI点迹个数(个):  %05d\n",tar_num);  
				for (jj = 0; jj<tar_num; jj++)
				{
					fseek(fid,offset+212+40*jj,SEEK_SET); 
					UINT32 point_no;
					fread(&point_no,sizeof(UINT32),1,fid);
					double tar_logn;
					fread(&tar_logn,sizeof(double),1,fid);
					double tar_lati;
					fread(&tar_lati,sizeof(double),1,fid);
					float tar_h;
					fread(&tar_h,sizeof(float),1,fid);
					float tar_vr;
					fread(&tar_vr,sizeof(float),1,fid);
					UINT32 loopNum;
					fread(&loopNum,sizeof(UINT32),1,fid);
					UINT32 rg_loc;
					fread(&rg_loc,sizeof(UINT32),1,fid);
					UINT32 az_loc;
					fread(&az_loc,sizeof(UINT32),1,fid);

					fprintf(fidtxt,"\n点迹编号: %04d\n",point_no);
                    fprintf(fidtxt,"点迹经度(度): %.7f\n",tar_logn);
                    fprintf(fidtxt,"点迹纬度(度): %.7f\n",tar_lati);
                    fprintf(fidtxt,"点迹高度(米): %.3f\n",tar_h);
                    fprintf(fidtxt,"径向速度(米/秒): %.3f\n",tar_vr);
					fprintf(fidtxt,"对应图像周期号: %05d\n",loopNum);
					fprintf(fidtxt,"点迹距离门: %d\n",rg_loc);
					fprintf(fidtxt,"点迹方位门: %d\n",az_loc);
				}
			}

		}

	   fclose(fid);
	   fclose(fidtxt);
	   delete [] pos;
       printf("File transfering is finished\n");
	
}

		void commonFuncs::ReadXL_EMapFile_mode_2(char* filename,char* outputdir)
 {
		
	     // 读取导引头位置信息
		 FILE *fid = NULL;
		 FILE *fidtxt = NULL;
		 
		 __int64 M,N,offset,len,kk,jj;
		 double lati,logn;
		 int track_num, tar_num;
		 int state = 0;
		 
		  fid = fopen(filename,"rb");
		  if(fid==NULL)
		  {
			  printf("Cannot open this file\n");
			  return;
		  }
		  fseek(fid,0,SEEK_END);
		  M = ftell(fid);
		  fseek(fid,0,SEEK_SET);

		  unsigned char* info = new unsigned char[M];
		  fread(info,sizeof(unsigned char),M,fid);
		  fclose(fid);
		 
		  long*  pos = new long[2000000];
		  int cnt = 0;
		  long prev_no = -1;
		  for(kk=0;kk<M-6;kk++)
		  {
			  if(info[kk] == 170 && info[kk+1] == 85 && info[kk+2] == 170 && info[kk+3]==85)
			  {
				  if(cnt > 199999) break;
				  pos[cnt] = kk;
				  cnt++;
			  }
		  }
		  delete [] info;
	
		  //Read target information
		  fid = fopen(filename,"rb");
		  printf("============ MTI mode moving targets track output ============\n");
		  printf("%s\n",filename);
		  N = cnt;
		  if(N==0) return;
	      for(kk=0;kk<N;kk++)
	      {
			char txtfile[512];
		    offset = pos[kk];
            fseek(fid,offset,SEEK_SET);
		    UINT32 len[3];
            fread(len,sizeof(UINT32),3,fid);
            UINT32 mode = len[1];
		    if(mode == 2)
		    {
				track_num = int((len[2]-216)/36);
		    }
		    
			if(mode == 1)
		    {
				tar_num = int((len[2]-216)/28);  
		    }

			if(mode == 3)
			{
				tar_num = int((len[2]-216)/40); 
			}
		   
			if(mode <= 0 || mode >3) return;

		    //read information
		    UINT16 taskcode;
			fread(&taskcode,sizeof(UINT16),1,fid);
			UINT8 planeType;
			fread(&planeType,sizeof(UINT8),1,fid);
			UINT8 planeNum;
			fread(&planeNum,sizeof(UINT8),1,fid);
			UINT16 planenum;
			fread(&planenum,sizeof(UINT16),1,fid);
			UINT8 pic_compresee_ratio;
			fread(&pic_compresee_ratio,sizeof(UINT8),1,fid);
			UINT8 transType;
			fread(&transType,sizeof(UINT8),1,fid);
			UINT8 loadType;
			fread(&loadType,sizeof(UINT8),1,fid);
			UINT32 loadNum;
			fread(&loadNum,sizeof(UINT32),1,fid);
			UINT16 openTimes;
			fread(&openTimes,sizeof(UINT16),1,fid);
			UINT32 IMUnum;
			fread(&IMUnum,sizeof(UINT32),1,fid);

		    UINT16 year;
			fread(&year,sizeof(UINT16),1,fid);
			UINT8 month;
			fread(&month,sizeof(UINT8),1,fid);
			UINT8 day;
			fread(&day,sizeof(UINT8),1,fid);
			UINT8 hour;
			fread(&hour,sizeof(UINT8),1,fid);
			UINT8 min;
			fread(&min,sizeof(UINT8),1,fid);
			UINT8 sec;
			fread(&sec,sizeof(UINT8),1,fid);
			UINT16 msec;
			fread(&msec,sizeof(UINT16),1,fid);

			double plane_logn;
			fread(&plane_logn,sizeof(double),1,fid);
			double plane_lati;
			fread(&plane_lati,sizeof(double),1,fid);

			float plane_height;
			fread(&plane_height,sizeof(float),1,fid);
			float plane_point_height;
			fread(&plane_point_height,sizeof(float),1,fid);
			
			float plane_track_angle;
			fread(&plane_track_angle,sizeof(float),1,fid);
			float plane_track_angle_v;
			fread(&plane_track_angle_v,sizeof(float),1,fid);
			float plane_track_angle_a;
			fread(&plane_track_angle_a,sizeof(float),1,fid);

			float plane_pitch_angle;
			fread(&plane_pitch_angle,sizeof(float),1,fid);
			float plane_pitch_angle_v;
			fread(&plane_pitch_angle_v,sizeof(float),1,fid);
			float plane_pitch_angle_a;
			fread(&plane_pitch_angle_a,sizeof(float),1,fid);

			float plane_roll_angle;
			fread(&plane_roll_angle,sizeof(float),1,fid);
			float plane_roll_angle_v;
			fread(&plane_roll_angle_v,sizeof(float),1,fid);
			float plane_roll_angle_a;
			fread(&plane_roll_angle_a,sizeof(float),1,fid);

			float plane_drift_angle;
			fread(&plane_drift_angle,sizeof(float),1,fid);
			float plane_crab_angle;
			fread(&plane_crab_angle,sizeof(float),1,fid);

			float ground_v;
			fread(&ground_v,sizeof(float),1,fid);
			float true_airspeed;
			fread(&true_airspeed,sizeof(float),1,fid);
			float indicator_airspeed;
			fread(&indicator_airspeed,sizeof(float),1,fid);
			float planespeed_east;
			fread(&planespeed_east,sizeof(float),1,fid);
			float planespeed_north;
			fread(&planespeed_north,sizeof(float),1,fid);
			float planespeed_sky;
			fread(&planespeed_sky,sizeof(float),1,fid);
			float plane_a_speed_east;
			fread(&plane_a_speed_east,sizeof(float),1,fid);
			float plane_a_speed_north;
			fread(&plane_a_speed_north,sizeof(float),1,fid);
			float plane_a_speed_sky;
			fread(&plane_a_speed_sky,sizeof(float),1,fid);

			UINT32 frameno;
			fread(&frameno,sizeof(UINT32),1,fid);
			UINT16 wavenum;
			fread(&wavenum,sizeof(UINT16),1,fid);
			UINT16 waveno;
			fread(&waveno,sizeof(UINT16),1,fid);

			UINT8 freq;
			fread(&freq,sizeof(UINT8),1,fid);
			UINT8 sideway;
			fread(&sideway,sizeof(UINT8),1,fid);
			UINT8 workmode;
			fread(&workmode,sizeof(UINT8),1,fid);
			UINT8 worksubmode;
			fread(&worksubmode,sizeof(UINT8),1,fid);

			UINT32 r_far;
			fread(&r_far,sizeof(UINT32),1,fid);
			UINT32 r_near;
			fread(&r_near,sizeof(UINT32),1,fid);

			float ScanCenterAng;
			fread(&ScanCenterAng,sizeof(float),1,fid);
			float ScanScope;
			fread(&ScanScope,sizeof(float),1,fid);
			float azimuthCenterAngle;
			fread(&azimuthCenterAngle,sizeof(float),1,fid);
			float beam_horz_width;
			fread(&beam_horz_width,sizeof(float),1,fid);
			float beam_scan_step;
			fread(&beam_scan_step,sizeof(float),1,fid);
			float lookangle;
			fread(&lookangle,sizeof(float),1,fid);
			float beam_R_width;
			fread(&beam_R_width,sizeof(float),1,fid);
			float beam_R_step;
			fread(&beam_R_step,sizeof(float),1,fid);

			UINT16 pulseResident;
			fread(&pulseResident,sizeof(UINT16),1,fid);
			UINT16 resTime;
			fread(&resTime,sizeof(UINT16),1,fid);
			float resolution;
			fread(&resolution,sizeof(float),1,fid);
			UINT32 prf;
			fread(&prf,sizeof(UINT32),1,fid);

			if(frameno != prev_no  || mode == 3) // SAR-GMTI
			{
				long year_s = year;
				if(year > 2000) year_s = year - 2000;
				memset(txtfile,0,sizeof(char)*512);
				sprintf(txtfile,"%s%04d_%02d%02d%02d_%02d%02d%02d_01_%05d_%03d_%04d_M%02d_%02d%02d%02d.txt",outputdir,taskcode,year_s,month,day,hour,min,sec,
				frameno,openTimes,planenum,mode,planeType,loadType,workmode);
				if(fidtxt !=NULL) {fclose(fidtxt); fidtxt=NULL;}
				fidtxt = fopen(txtfile,"a");
			}

			fprintf(fidtxt,"\n\n信息标签:  %02d\n",mode);
			fprintf(fidtxt,"任务代号:  %04d\n",taskcode);
            fprintf(fidtxt,"飞行器类型:  %02d\n",planeType);
            fprintf(fidtxt,"飞机批号:  %02d\n",planeNum);
            fprintf(fidtxt,"飞机号:  %04d\n",planenum);
            fprintf(fidtxt,"图像压缩比:  %02d\n",pic_compresee_ratio);
            fprintf(fidtxt,"传输方式(0=地面回读,1=实时传输,2=选择传输,3=精细成像):  %d\n",transType);
            fprintf(fidtxt,"载荷类型(01-高空CCD, 02-长焦倾斜CCD，03-SAR，04-红外行扫仪，05-多光谱):  %02d\n",loadType);
            fprintf(fidtxt,"载荷编号:  %06d\n",loadNum);
            fprintf(fidtxt,"开机次数:  %03d\n",openTimes);
            fprintf(fidtxt,"惯导序号:  %05d\n",IMUnum);

			long year_s = year;
			if(year < 2000) year_s = year + 2000;
            fprintf(fidtxt,"日期:  %04d.%02d.%02d\n",year_s,month,day);
            fprintf(fidtxt,"时间:  %02d-%02d-%02d-%03d\n",hour,min,sec,msec);
            fprintf(fidtxt,"飞机经度(度):  %.7f\n",plane_logn);
            fprintf(fidtxt,"飞机纬度(度):  %.7f\n",plane_lati);
            fprintf(fidtxt,"飞机海拔高度(米):  %.3f\n",plane_height);
            fprintf(fidtxt,"机下点海拔高度(米):  %.3f\n",plane_point_height);
            fprintf(fidtxt,"飞机航向角(度):  %.7f\n",plane_track_angle);
            fprintf(fidtxt,"飞机航向角速度(度/秒):  %.3f\n",plane_track_angle_v);
            fprintf(fidtxt,"飞机航向角加速度(度/平方秒):  %.3f\n",plane_track_angle_a);
            fprintf(fidtxt,"飞机俯仰向角(度)  %.7f\n",plane_pitch_angle);
            fprintf(fidtxt,"飞机俯仰向角速度(度/秒): 0.000\n");
            fprintf(fidtxt,"飞机俯仰向角加速度(度/平方秒):  0.000\n");
            fprintf(fidtxt,"飞机横滚角(度):  %.7f\n",plane_roll_angle);
            fprintf(fidtxt,"飞机横滚角速度(度/秒):  0.000\n");
            fprintf(fidtxt,"飞机横滚角加速度(度/平方秒):  0.000\n");
            fprintf(fidtxt,"飞机偏航角:  %.7f\n",plane_drift_angle );
            fprintf(fidtxt,"飞机偏流角(度):  0.0000000\n");
            fprintf(fidtxt,"地速(米/秒):  %.4f\n",ground_v );
            fprintf(fidtxt,"真空速(米/秒):  0.0000\n");
            fprintf(fidtxt,"指示空速(米/秒):  0.0000\n"); 
            fprintf(fidtxt,"飞机东速(米/秒):  %.4f\n",planespeed_east );
            fprintf(fidtxt,"飞机北速(米/秒):  %.4f\n",planespeed_north );
            fprintf(fidtxt,"飞机天速(米/秒):  %.4f\n",planespeed_sky);  
            fprintf(fidtxt,"东向加速度(米/秒平方):  %.4f\n",plane_a_speed_east );
            fprintf(fidtxt,"北向加速度(米/秒平方):  %.4f\n",plane_a_speed_north);
            fprintf(fidtxt,"天向加速度(米/秒平方):  %.4f\n",plane_a_speed_sky);
            fprintf(fidtxt,"天线帧编号:  %06d\n",frameno);
            fprintf(fidtxt,"波位数(个):  %04d\n",wavenum);
            fprintf(fidtxt,"波位号:  %04d\n",waveno);
            fprintf(fidtxt,"工作频段(0=X，1=L，2=P，3=P+L，4=Ku):  %02d\n",freq);
            fprintf(fidtxt,"侧视方式(0=左侧视，1=右侧视):  %02d\n",sideway);
            fprintf(fidtxt,"工作模式(0=GMTI，1=MMTI，2=AMTI，3=SAR/MTI同时模式):  %02d\n",workmode);
            fprintf(fidtxt,"工作子模式(0=广域，1=扇区，2=跟踪，9=空缺): %02d\n",worksubmode);
            fprintf(fidtxt,"最大作用距离(米):  %d\n",r_far);
            fprintf(fidtxt,"最小作用距离(米):  %d\n",r_near);

            fprintf(fidtxt,"天线帧扫描中心角(度):  %.4f\n",ScanCenterAng);
            fprintf(fidtxt,"天线帧扫描范围(度):  %.4f\n",ScanScope);
            fprintf(fidtxt,"方位波束中心角(度):  %.4f\n",azimuthCenterAngle);
            fprintf(fidtxt,"方位波束宽度(度):  %.4f\n",beam_horz_width);
            fprintf(fidtxt,"方位向扫描步进(度):  %.4f\n",beam_scan_step);
            fprintf(fidtxt,"俯仰波束中心角(度):  %.4f\n",lookangle);
            fprintf(fidtxt,"俯仰波束宽度(度):  %.4f\n",beam_R_width);
            fprintf(fidtxt,"俯仰向扫描步进(度):  %.4f\n",beam_R_step);
            fprintf(fidtxt,"驻留脉冲数(个):  %04d\n",pulseResident);
            fprintf(fidtxt,"波束驻留时间(毫秒):  %05d\n",resTime);
            fprintf(fidtxt,"分辨率(米):  %.1f\n",resolution);
            fprintf(fidtxt,"重复频率(Hz):  %05d\n",prf);  

			//写航迹文件
			if(mode==2)  //output track information 
			{
				fprintf(fidtxt,"航迹个数(个):  %05d\n",track_num);  
				for (jj = 0; jj<track_num; jj++)
				{
					fseek(fid,offset+212+36*jj,SEEK_SET); 
					UINT32 track_no;
					fread(&track_no,sizeof(UINT32),1,fid);
					double tar_gps[2];
					fread(tar_gps,sizeof(double),2,fid);
					float tar_high;
					fread(&tar_high,sizeof(float),1,fid);
					float tar_vr;
					fread(&tar_vr,sizeof(float),1,fid);
					float tar_dir;
					fread(&tar_dir,sizeof(float),1,fid);

					fprintf(fidtxt,"\n航迹批号: %04d\n",track_no);
                    fprintf(fidtxt,"目标经度(度): %.7f\n",tar_gps[0]);
                    fprintf(fidtxt,"目标纬度(度): %.7f\n",tar_gps[1]);
                    fprintf(fidtxt,"目标高度(米): %.3f\n",tar_high);
                    fprintf(fidtxt,"目标速度(米/秒): %.3f\n",tar_vr);
                    fprintf(fidtxt,"目标方向(度): %.4f\n",tar_dir);
                    fprintf(fidtxt,"新批标识(1=是,0=否): 1\n");
                    fprintf(fidtxt,"目标属性(0=敌,1=我,2=不明): 2\n");
				}
			}

			//写点迹文件
			if(mode==1)  //output track information 
			{
				fprintf(fidtxt,"点迹个数(个):  %05d\n",tar_num);  
				for (jj = 0; jj<tar_num; jj++)
				{
					fseek(fid,offset+212+28*jj,SEEK_SET); 
					UINT32 point_no;
					fread(&point_no,sizeof(UINT32),1,fid);
					double tar_logn;
					fread(&tar_logn,sizeof(double),1,fid);
					double tar_lati;
					fread(&tar_lati,sizeof(double),1,fid);
					float tar_h;
					fread(&tar_h,sizeof(float),1,fid);
					float tar_vr;
					fread(&tar_vr,sizeof(float),1,fid);

					fprintf(fidtxt,"\n点迹编号: %04d\n",point_no);
                    fprintf(fidtxt,"点迹经度(度): %.7f\n",tar_logn);
                    fprintf(fidtxt,"点迹纬度(度): %.7f\n",tar_lati);
                    fprintf(fidtxt,"点迹高度(米): %.3f\n",tar_h);
                    fprintf(fidtxt,"径向速度(米/秒): %.3f\n",tar_vr);
				}
			}

			//写点迹文件
			if(mode==3)  //output track information 
			{
				fprintf(fidtxt,"SAR-GMTI点迹个数(个):  %05d\n",tar_num);  
				for (jj = 0; jj<tar_num; jj++)
				{
					fseek(fid,offset+212+40*jj,SEEK_SET); 
					UINT32 point_no;
					fread(&point_no,sizeof(UINT32),1,fid);
					double tar_logn;
					fread(&tar_logn,sizeof(double),1,fid);
					double tar_lati;
					fread(&tar_lati,sizeof(double),1,fid);
					float tar_h;
					fread(&tar_h,sizeof(float),1,fid);
					float tar_vr;
					fread(&tar_vr,sizeof(float),1,fid);
					UINT32 loopNum;
					fread(&loopNum,sizeof(UINT32),1,fid);
					UINT32 rg_loc;
					fread(&rg_loc,sizeof(UINT32),1,fid);
					UINT32 az_loc;
					fread(&az_loc,sizeof(UINT32),1,fid);

					fprintf(fidtxt,"\n点迹编号: %04d\n",point_no);
                    fprintf(fidtxt,"点迹经度(度): %.7f\n",tar_logn);
                    fprintf(fidtxt,"点迹纬度(度): %.7f\n",tar_lati);
                    fprintf(fidtxt,"点迹高度(米): %.3f\n",tar_h);
                    fprintf(fidtxt,"径向速度(米/秒): %.3f\n",tar_vr);
					fprintf(fidtxt,"对应图像周期号: %05d\n",loopNum);
					fprintf(fidtxt,"点迹距离门: %d\n",rg_loc);
					fprintf(fidtxt,"点迹方位门: %d\n",az_loc);
				}
			}

			prev_no = frameno;   	
	   }

	   fclose(fid);
	   if(fidtxt !=NULL) {fclose(fidtxt); fidtxt=NULL;}
	   delete [] pos;
       printf("File transfering is finished\n");
	
}

		void commonFuncs::ReadXL_EMapFile_mode_3(char* filename,char* outputdir)
 {
		
	 //1.读取导引头位置信息
		 FILE *fid = NULL;
		 FILE *fidtxt = NULL;

		 __int64 M,N,offset,len,kk,jj;
		 double lati,logn;
		 int track_num, tar_num;
		 int state = 0;
		 
		  fid = fopen(filename,"rb");
		  if(fid==NULL)
		  {
			  printf("Cannot open this file\n");
			  return;
		  }
		  fseek(fid,0,SEEK_END);
		  M = ftell(fid);
		  fseek(fid,0,SEEK_SET);

		  unsigned char* info = new unsigned char[M];
		  fread(info,sizeof(unsigned char),M,fid);
		  fclose(fid);
		 
		  long*  pos = new long[2000000];
		  int cnt = 0;
		  long prev_no = -1;
		  for(kk=0;kk<M-6;kk++)
		  {
			  if(info[kk] == 170 && info[kk+1] == 85 && info[kk+2] == 170 && info[kk+3]==85)
			  {
				  if(cnt > 1999999) break;
				  pos[cnt] = kk;
				  cnt++;
			  }
		  }
		  delete [] info;
		  if(cnt==0) return;
	
		  //Read target information
		  fid = fopen(filename,"rb");
		  printf("============ MTI mode moving targets track output ============\n");
		  printf("%s\n",filename);
		  N = cnt;
		  if(N==0) return;

		  char txtfile[512];
	  for(kk=0;kk<N;kk++)
	  {
		   offset = pos[kk];
           fseek(fid,offset,SEEK_SET);
		   UINT32 len[3];

		   if(feof(fid)) break;

           fread(len,sizeof(UINT32),3,fid);
           UINT32 mode = len[1];
		   
		   if(mode == 2)
		   {
			   track_num = int((len[2]-216)/36);
		   }
		   else continue;
		   
		   //read information
		    UINT16 taskcode;
			fread(&taskcode,sizeof(UINT16),1,fid);
			UINT8 planeType;
			fread(&planeType,sizeof(UINT8),1,fid);
			UINT8 planeNum;
			fread(&planeNum,sizeof(UINT8),1,fid);
			UINT16 planenum;
			fread(&planenum,sizeof(UINT16),1,fid);
			UINT8 pic_compresee_ratio;
			fread(&pic_compresee_ratio,sizeof(UINT8),1,fid);
			UINT8 transType;
			fread(&transType,sizeof(UINT8),1,fid);
			UINT8 loadType;
			fread(&loadType,sizeof(UINT8),1,fid);
			UINT32 loadNum;
			fread(&loadNum,sizeof(UINT32),1,fid);
			UINT16 openTimes;
			fread(&openTimes,sizeof(UINT16),1,fid);
			UINT32 IMUnum;
			fread(&IMUnum,sizeof(UINT32),1,fid);

		    UINT16 year;
			fread(&year,sizeof(UINT16),1,fid);
			UINT8 month;
			fread(&month,sizeof(UINT8),1,fid);
			UINT8 day;
			fread(&day,sizeof(UINT8),1,fid);
			UINT8 hour;
			fread(&hour,sizeof(UINT8),1,fid);
			UINT8 min;
			fread(&min,sizeof(UINT8),1,fid);
			UINT8 sec;
			fread(&sec,sizeof(UINT8),1,fid);
			UINT16 msec;
			fread(&msec,sizeof(UINT16),1,fid);

			double plane_logn;
			fread(&plane_logn,sizeof(double),1,fid);
			double plane_lati;
			fread(&plane_lati,sizeof(double),1,fid);

			float plane_height;
			fread(&plane_height,sizeof(float),1,fid);
			float plane_point_height;
			fread(&plane_point_height,sizeof(float),1,fid);
			
			float plane_track_angle;
			fread(&plane_track_angle,sizeof(float),1,fid);
			float plane_track_angle_v;
			fread(&plane_track_angle_v,sizeof(float),1,fid);
			float plane_track_angle_a;
			fread(&plane_track_angle_a,sizeof(float),1,fid);

			float plane_pitch_angle;
			fread(&plane_pitch_angle,sizeof(float),1,fid);
			float plane_pitch_angle_v;
			fread(&plane_pitch_angle_v,sizeof(float),1,fid);
			float plane_pitch_angle_a;
			fread(&plane_pitch_angle_a,sizeof(float),1,fid);

			float plane_roll_angle;
			fread(&plane_roll_angle,sizeof(float),1,fid);
			float plane_roll_angle_v;
			fread(&plane_roll_angle_v,sizeof(float),1,fid);
			float plane_roll_angle_a;
			fread(&plane_roll_angle_a,sizeof(float),1,fid);

			float plane_drift_angle;
			fread(&plane_drift_angle,sizeof(float),1,fid);
			float plane_crab_angle;
			fread(&plane_crab_angle,sizeof(float),1,fid);

			float ground_v;
			fread(&ground_v,sizeof(float),1,fid);
			float true_airspeed;
			fread(&true_airspeed,sizeof(float),1,fid);
			float indicator_airspeed;
			fread(&indicator_airspeed,sizeof(float),1,fid);
			float planespeed_east;
			fread(&planespeed_east,sizeof(float),1,fid);
			float planespeed_north;
			fread(&planespeed_north,sizeof(float),1,fid);
			float planespeed_sky;
			fread(&planespeed_sky,sizeof(float),1,fid);
			float plane_a_speed_east;
			fread(&plane_a_speed_east,sizeof(float),1,fid);
			float plane_a_speed_north;
			fread(&plane_a_speed_north,sizeof(float),1,fid);
			float plane_a_speed_sky;
			fread(&plane_a_speed_sky,sizeof(float),1,fid);

			UINT32 frameno;
			fread(&frameno,sizeof(UINT32),1,fid);
			UINT16 wavenum;
			fread(&wavenum,sizeof(UINT16),1,fid);
			UINT16 waveno;
			fread(&waveno,sizeof(UINT16),1,fid);

			UINT8 freq;
			fread(&freq,sizeof(UINT8),1,fid);
			UINT8 sideway;
			fread(&sideway,sizeof(UINT8),1,fid);
			UINT8 workmode;
			fread(&workmode,sizeof(UINT8),1,fid);
			UINT8 worksubmode;
			fread(&worksubmode,sizeof(UINT8),1,fid);

			UINT32 r_far;
			fread(&r_far,sizeof(UINT32),1,fid);
			UINT32 r_near;
			fread(&r_near,sizeof(UINT32),1,fid);

			float ScanCenterAng;
			fread(&ScanCenterAng,sizeof(float),1,fid);
			float ScanScope;
			fread(&ScanScope,sizeof(float),1,fid);
			ScanScope = ScanScope/2;
			float azimuthCenterAngle;
			fread(&azimuthCenterAngle,sizeof(float),1,fid);
			float beam_horz_width;
			fread(&beam_horz_width,sizeof(float),1,fid);
			float beam_scan_step;
			fread(&beam_scan_step,sizeof(float),1,fid);
			float lookangle;
			fread(&lookangle,sizeof(float),1,fid);
			float beam_R_width;
			fread(&beam_R_width,sizeof(float),1,fid);
			float beam_R_step;
			fread(&beam_R_step,sizeof(float),1,fid);

			UINT16 pulseResident;
			fread(&pulseResident,sizeof(UINT16),1,fid);
			UINT16 resTime;
			fread(&resTime,sizeof(UINT16),1,fid);
			float resolution;
			fread(&resolution,sizeof(float),1,fid);
			UINT32 prf;
			fread(&prf,sizeof(UINT32),1,fid);

			if (state == 0)
			{
				if(fidtxt != NULL){fidtxt = NULL;}
				memset(txtfile,0,sizeof(char)*512);
				long year_s = year;
				if(year > 2000) year_s = year - 2000;
				sprintf(txtfile,"%s%04d_%02d%02d%02d_%02d%02d%02d_01_%05d_%03d_%04d_M%02d_%02d%02d%02d.txt",outputdir,taskcode,year_s,month,day,hour,min,sec,
				frameno,openTimes,planenum,mode,planeType,loadType,workmode);
				fidtxt = fopen(txtfile,"a");
				if(fidtxt == NULL){printf("open txtfile fail !\n");return;}
				state = state + 1;
			}
			        fprintf(fidtxt,"\n\n信息标签:  %02d\n",mode);
                    fprintf(fidtxt,"任务代号:  %04d\n",taskcode);
                    fprintf(fidtxt,"飞行器类型:  %02d\n",planeType);
                    fprintf(fidtxt,"飞机批号:  %02d\n",planeNum);
                    fprintf(fidtxt,"飞机号:  %04d\n",planenum);
                    fprintf(fidtxt,"图像压缩比:  %02d\n",pic_compresee_ratio);
                    fprintf(fidtxt,"传输方式(0=地面回读,1=实时传输,2=选择传输,3=精细成像):  %d\n",transType);
                    fprintf(fidtxt,"载荷类型(01-高空CCD, 02-长焦倾斜CCD，03-SAR，04-红外行扫仪，05-多光谱):  %02d\n",loadType);
                    fprintf(fidtxt,"载荷编号:  %06d\n",loadNum);
                    fprintf(fidtxt,"开机次数:  %03d\n",openTimes);
                    fprintf(fidtxt,"惯导序号:  %05d\n",IMUnum);

					long year_s = year;
					if(year < 2000) year_s = year + 2000;
					fprintf(fidtxt,"日期:  %04d.%02d.%02d\n",year_s,month,day);
                    fprintf(fidtxt,"时间:  %02d-%02d-%02d-%03d\n",hour,min,sec,msec);
                    fprintf(fidtxt,"飞机经度(度):  %.7f\n",plane_logn);
                    fprintf(fidtxt,"飞机纬度(度):  %.7f\n",plane_lati);
                    fprintf(fidtxt,"飞机海拔高度(米):  %.3f\n",plane_height);
                    fprintf(fidtxt,"机下点海拔高度(米):  %.3f\n",plane_height);
                    fprintf(fidtxt,"飞机航向角(度):  %.7f\n",plane_track_angle);
                    fprintf(fidtxt,"飞机航向角速度(度/秒):  %.3f\n",plane_track_angle_v);
                    fprintf(fidtxt,"飞机航向角加速度(度/平方秒):  %.3f\n",plane_track_angle_a);
                    fprintf(fidtxt,"飞机俯仰向角(度)  %.7f\n",plane_pitch_angle);
                    fprintf(fidtxt,"飞机俯仰向角速度(度/秒): 0.000\n");
                    fprintf(fidtxt,"飞机俯仰向角加速度(度/平方秒):  0.000\n");
                    fprintf(fidtxt,"飞机横滚角(度):  %.7f\n",plane_roll_angle);
                    fprintf(fidtxt,"飞机横滚角速度(度/秒):  0.000\n");
                    fprintf(fidtxt,"飞机横滚角加速度(度/平方秒):  0.000\n");
                    fprintf(fidtxt,"飞机偏航角:  %.7f\n",plane_drift_angle );
                    fprintf(fidtxt,"飞机偏流角(度):  0.0000000\n");
                    fprintf(fidtxt,"地速(米/秒):  %.4f\n",ground_v );
                    fprintf(fidtxt,"真空速(米/秒):  0.0000\n");
                    fprintf(fidtxt,"指示空速(米/秒):  0.0000\n"); 
                    fprintf(fidtxt,"飞机东速(米/秒):  %.4f\n",planespeed_east );
                    fprintf(fidtxt,"飞机北速(米/秒):  %.4f\n",planespeed_north );
                    fprintf(fidtxt,"飞机天速(米/秒):  %.4f\n",planespeed_sky);  
                    fprintf(fidtxt,"东向加速度(米/秒平方):  %.4f\n",plane_a_speed_east );
                    fprintf(fidtxt,"北向加速度(米/秒平方):  %.4f\n",plane_a_speed_north);
                    fprintf(fidtxt,"天向加速度(米/秒平方):  %.4f\n",plane_a_speed_sky);
                    fprintf(fidtxt,"天线帧编号:  %06d\n",frameno);
                    fprintf(fidtxt,"波位数(个):  %04d\n",wavenum);
                    fprintf(fidtxt,"波位号:  %04d\n",waveno);
                    fprintf(fidtxt,"工作频段(0=X，1=L，2=P，3=P+L，4=Ku):  %02d\n",freq);
                    fprintf(fidtxt,"侧视方式(0=左侧视，1=右侧视):  %02d\n",sideway);
                    fprintf(fidtxt,"工作模式(0=GMTI，1=MMTI，2=AMTI，3=SAR/MTI同时模式):  %02d\n",workmode);
                    fprintf(fidtxt,"工作子模式(0=广域，1=扇区，2=跟踪，9=空缺): %02d\n",worksubmode);
                    fprintf(fidtxt,"最大作用距离(米):  %d\n",r_far);
                    fprintf(fidtxt,"最小作用距离(米):  %d\n",r_near);
 
                    fprintf(fidtxt,"天线帧扫描中心角(度):  %.4f\n",ScanCenterAng);
                    fprintf(fidtxt,"天线帧扫描范围(度):  %.4f\n",ScanScope);
                    fprintf(fidtxt,"方位波束中心角(度):  %.4f\n",azimuthCenterAngle);
                    fprintf(fidtxt,"方位波束宽度(度):  %.4f\n",beam_horz_width);
                    fprintf(fidtxt,"方位向扫描步进(度):  %.4f\n",beam_scan_step);
                    fprintf(fidtxt,"俯仰波束中心角(度):  %.4f\n",lookangle);
                    fprintf(fidtxt,"俯仰波束宽度(度):  %.4f\n",beam_R_width);
                    fprintf(fidtxt,"俯仰向扫描步进(度):  %.4f\n",beam_R_step);
                    fprintf(fidtxt,"驻留脉冲数(个):  %04d\n",pulseResident);
                    fprintf(fidtxt,"波束驻留时间(毫秒):  %05d\n",resTime);
                    fprintf(fidtxt,"分辨率(米):  %.1f\n",resolution);
                    fprintf(fidtxt,"重复频率(Hz):  %05d\n",prf);  

					//写航迹文件
				if(mode==2)  //output track information 
				{
                    fprintf(fidtxt,"航迹个数(个):  %05d\n",track_num);  
					for (jj = 0; jj<track_num; jj++)
					{
						fseek(fid,offset+212+36*jj,SEEK_SET); 
						UINT32 track_no;
						fread(&track_no,sizeof(UINT32),1,fid);
						double tar_gps[2];
						fread(tar_gps,sizeof(double),2,fid);
						float tar_high;
						fread(&tar_high,sizeof(float),1,fid);
						float tar_vr;
						fread(&tar_vr,sizeof(float),1,fid);
						float tar_dir;
						fread(&tar_dir,sizeof(float),1,fid);

						fprintf(fidtxt,"\n航迹批号: %04d\n",track_no);
                        fprintf(fidtxt,"目标经度(度): %.7f\n",tar_gps[0]);
                        fprintf(fidtxt,"目标纬度(度): %.7f\n",tar_gps[1]);
                        fprintf(fidtxt,"目标高度(米): %.3f\n",tar_high);
                        fprintf(fidtxt,"目标速度(米/秒): %.3f\n",tar_vr);
                        fprintf(fidtxt,"目标方向(度): %.4f\n",tar_dir);
                        fprintf(fidtxt,"新批标识(1=是,0=否): 1\n");
                        fprintf(fidtxt,"目标属性(0=敌,1=我,2=不明): 2\n");
					}
				}
	   }

	   if(fid != NULL) fclose(fid);
	   if(fidtxt != NULL) fclose(fidtxt);
	   if(pos != NULL) delete [] pos; pos = NULL;
       printf("File transfering is finished\n");
}

		// 动目标图像标记类

	void ImgMark::SAR_GmtiTar_Mark(char *szMarkFile,char *szGrayFile,unsigned char *imgData,long *tar_az,long *tar_ra, char *tar_flag, float *tarVr,__int64 nImgWidth,__int64 nImgHeight,int nTarNum, int lookside, double *img_latis, double *img_logns)
	{
		const unsigned char NUMBER7x9[13][9] = 
		{
			{0x00,0x1C,0x22,0x22,0x22,0x22,0x22,0x1C,0x00},/*0*/
			{0x00,0x20,0x30,0x20,0x20,0x20,0x20,0x20,0x00},/*1*/
			{0x00,0x1E,0x20,0x20,0x1C,0x02,0x02,0x3E,0x00},/*2*/
			{0x00,0x1E,0x20,0x20,0x1C,0x20,0x20,0x1E,0x00},/*3*/
			{0x00,0x10,0x18,0x14,0x12,0x3E,0x10,0x10,0x00},/*4*/
			{0x00,0x3E,0x02,0x02,0x1E,0x20,0x20,0x1E,0x00},/*5*/
			{0x00,0x1C,0x02,0x02,0x1E,0x22,0x22,0x1C,0x00},/*6*/
			{0x00,0x3E,0x20,0x10,0x10,0x08,0x08,0x08,0x00},/*7*/
			{0x00,0x1C,0x22,0x22,0x1C,0x22,0x22,0x1C,0x00},/*8*/
			{0x00,0x1C,0x22,0x22,0x1C,0x20,0x20,0x1C,0x00},/*9*/
			{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00},/*.*/
			{0x00,0x00,0x00,0x00,0x1C,0x00,0x00,0x00,0x00},/*-*/
			{0x00,0x3E,0x08,0x08,0x08,0x08,0x08,0x08,0x00},/*T*/	
		};

	// 字体高度是9, 每行有4个字
	unsigned char numMaskGrayH[36], numMaskGrayL[36];

	__int64 h, w, m, n;
	int nMarkHeight = 42;  // 9 + 24 + 9
	int nMarkWidth  = 24;  // 4 * 6

	// unsigned char *imgGray = new unsigned char[nImgWidth * nImgHeight];
	// memcpy(imgGray,imgData,sizeof(unsigned char)*__int64(nImgWidth) * nImgHeight);
	
	// image correction 
	imgProc im;
	im.ImageCorr_for6suoMap(imgData,nImgHeight,nImgWidth,lookside);
	
	// Create geo-tiff file // 需要修改imgGray
	// 根据需求输出灰度图文件
	if(szGrayFile[0] && GEOTIFF_VERSION)
	{
		GeoTiff geo;
		geo.WriteByte2Geotiff(img_latis,img_logns,nImgWidth,nImgHeight,imgData,szGrayFile);
	}

	// 根据需求输出灰度图文件
	if(szGrayFile[0] && !GEOTIFF_VERSION)
	{
		Form_Gray8_Tif(imgData, szGrayFile, nImgHeight, nImgWidth);
	}

	if(szMarkFile[0] == 0)
	{  
		// empty string
		// delete [] imgGray;	imgGray = NULL;
		return;
	}

	// 把灰度图变成RGB图
	unsigned char *imgRGB  = new unsigned char[nImgWidth * nImgHeight * 3];

	if(imgRGB == NULL) return;

	unsigned char gray;
	for(h=0; h<nImgHeight*nImgWidth; h++)
	{
		gray = *(imgData+h);
		*(imgRGB+3*h)   = gray;
		*(imgRGB+3*h+1) = gray;
		*(imgRGB+3*h+2) = gray;
	}

			 
	for(n = 0; n < nTarNum; n ++)
	{
		// 目标位置上下也要颠倒一下
		int tarPosH = nImgHeight - tar_az[n] - 1; // modified by luo
		int tarPosW;
		if(lookside <= 0)
			tarPosW = tar_ra[n];  // 右侧视
		else 
			tarPosW = nImgWidth - tar_ra[n] - 1;
		
		// 目标在图像边缘
		if(tarPosH > (nImgHeight - nMarkHeight/2 - 1))continue;
		if(tarPosH < (nMarkHeight/2 + 1))continue;
		if(tarPosW > (nImgWidth - nMarkWidth/2 - 1))continue;
		if(tarPosW < (nMarkWidth/2 + 1))continue;

		char vSting[20];

		// 上方字符串:Txxx
		sprintf_s(vSting, "T%03d", n );  // 编号从0开始--已修改
		for(int s = 0; s < 4; s ++){
			int offset = vSting[s] - '0';
			if(vSting[s] == 'T')offset = 12;

			for(m = 0; m < 9; m ++){
				numMaskGrayH[m * 4 + s] = NUMBER7x9[offset][m];
			}
		}

		// 下方字符串:速度
		sprintf_s(vSting, "%f", tarVr[n]);
		for(int s = 0; s < 4; s ++)
		{
			int offset = vSting[s] - '0';
			if(vSting[s] == '.')offset = 10;
			if(vSting[s] == '-')offset = 11;

			for(m = 0; m < 9; m ++){
				numMaskGrayL[m * 4 + s] = NUMBER7x9[offset][m];
			}
		}

		for(h = 0; h < nMarkHeight; h ++)
		{ 
			unsigned char *pLineRGB = imgRGB + (tarPosH + h - nMarkHeight/2) * 3 * nImgWidth + (tarPosW - nMarkWidth/2) * 3;

			if(h < 9)
			{
				for(w = 0; w < nMarkWidth; w ++){
					unsigned char nMask = numMaskGrayH[h * 4 + w / 6];
					nMask >>= (w % 6);
					if(nMask & 0x01){
						pLineRGB[3 * w + 0] = 0;
						pLineRGB[3 * w + 1] = 0;
						pLineRGB[3 * w + 2] = 255;
					}
				}
			}
			else if(h < 33)
			{  // target mark
				
				for(w = 0; w < nMarkWidth; w ++)
				{
					pLineRGB[3 * w + 0] = 255*int(tarVr[n]>0);
					pLineRGB[3 * w + 1] = 0;
					pLineRGB[3 * w + 2] = 255*int(tarVr[n]<0);
				}
			}
			else
			{
				for(w = 0; w < nMarkWidth; w ++)
				{
					unsigned char nMask = numMaskGrayL[(h - 33) * 4 + w / 6];
					nMask >>= (w % 6);
					if(nMask & 0x01)
					{
						pLineRGB[3 * w + 0] = 0;
						pLineRGB[3 * w + 1] = 0;
						pLineRGB[3 * w + 2] = 255;
					}
				}
			}

		}
	}

	Form_RGB8_Tif(imgRGB, szMarkFile, nImgHeight, nImgWidth);

	delete [] imgRGB; imgRGB = NULL;	
	
}

	template <class T>
	double ImgMark::min_val(T *val,__int64 num,__int64 *loc)
        {
           long    i;
	       long    min_loc=0;
	      double  min_val0;
		  if(num<0) return 0.0;
			min_val0=val[0];
	        for(i=0;i<num;i++)
	        {
	           if(val[i]<min_val0) 
	           {	  
	            min_val0=val[i];
		        min_loc=i;
	           }
	         }
	     *loc=min_loc;
	     return   min_val0;
     }
	
	template <class T>
	inline double ImgMark::mean_val(T *val,long num)
        {
           long    i;
	      double  av_val=0.0;
		   if(num<0) return 0.0;
		   for(i=0;i<num;i++) av_val+=double(val[i]);
	       av_val/=double(num);
	        return   av_val;
         }
	
	template <class T>
	inline  double ImgMark::std_val(T *val,long num)
        {
            long    i;
	        double  av_val=0.0,temp_val;
			double  dd_val=0.0;   

		    if(num<0) return 0.0;
		    for(i=0;i<num;i++)  
			{	
               temp_val=double(val[i]);  			
				av_val+=temp_val;
		        dd_val+=temp_val*temp_val;
			  }
			dd_val/=double(num);
		    av_val/=double(num);
			return   sqrt(dd_val-av_val*av_val);
     }

	template <class T>
	inline double ImgMark::max_val(T *val,long num,long *loc)
       {
         long    i;
	     long   max_loc=0;
	      double  max_val0;
	     if(num<0) return 0.0;
		 max_val0=val[0];
 	      for(i=0;i<num;i++)
	      {
	       if(val[i]>max_val0) 
	         {	  
	          max_val0=val[i];
	           max_loc=i;
	         }
	      }
	     *loc=max_loc;
	   return  max_val0;
      }	
	
	template <class T>
	inline double  ImgMark::sinc(T x)
    {
		double  val;
		if(x==0)  val=1.0;
		else      val=sin(PI*x)/x/PI;
		return val;

    }
	
	int ImgMark::SAR_FOPEN(FILE **fid, char *fn, char *mode)
		{
			*fid=fopen(fn, mode);
			if(fid ==NULL)
	   		{
				printf("file open failed: %s\n", fn);
				exit(-3);
			}
		return 0;
	  
		}
	
	int ImgMark::SAR_FREAD(FILE **fid, void *buffer, size_t size)
		{
 			if(fread(buffer, 1, size, *fid)!=size)
			{
	 		printf("file read error!\n");
			exit(-3);
			}
		return 0;
		}
	
	int ImgMark::SAR_FWRITE(FILE **fid, void *buffer, size_t size)
		{
			if(fwrite(buffer, 1, size, *fid)!=size)
			{
			printf("file write error!\n"); 
			exit(-3);
			}
			return 0;
		}
	
	int ImgMark::SAR_FCLOSE(FILE **fid)
		{
		if(fclose(*fid))
			{
			printf("file close error!\n");
			exit(-3);
			}
		return 0;
		}
	
	int ImgMark::SAR_FSEEK(FILE **fid, __int64 offset, int origin)
	{
		if(_fseeki64(*fid, offset, origin))
		{
		printf("file seek error!\n");
		exit(-3);
		}
    	return 0;
	}
	
	int ImgMark::Form_Gray8_Tif(unsigned char *img_data,char *img_file,long high,long wid)
    {//0420
		        FILE        *fp_img;
			   __int64          loc,n,m;
			   double           rat_ampl=1.0;
			   double            max_ampl,mean_ampl,std_ampl;
			   unsigned __int16  de_num=11;
			   unsigned __int16  tag[]={254,256,257,258,259,262,273,274,277,278,279};
			   unsigned __int16  type[]={4,4,4,3,3,3,4,3,3,4,4};
			   unsigned __int32  len[] ={1,1,1,1,1,1,1,1,1,1,1};
			   unsigned __int32  val_offset[]={0,15872,133424,8,1,1,168,1,1,133424,211111190};
			   unsigned __int8   ifh[]={73,73,42,0};
			   unsigned __int16  col[]={8,8,8};
			   unsigned __int32  res[]={720000,10000,720000,10000};
			   unsigned __int32   mov=8;
			   unsigned __int32   idl=0;

			   if((high*wid)<10) { printf("图像文件大小不对"); return 0;}

			   ////////////////////////////////////////////////////////////////			  
			     val_offset[1]=wid; 
				 val_offset[2]=high;
				 val_offset[9]=high;
				 val_offset[10]=wid*high;
			 /////////////////form tiff////////////////////////////////////
			 ////////////IFH FORM///////////////////////
			   SAR_FOPEN(&fp_img,img_file,"wb");
			   SAR_FWRITE(&fp_img,ifh,sizeof(unsigned char)*4);
			   SAR_FWRITE(&fp_img,&mov,sizeof(unsigned __int32));
			   ///////////////////IFD FORM//////////////////
			   SAR_FWRITE(&fp_img,&de_num,sizeof(unsigned __int16));
			   for(m=0;m<de_num;m++)
			   {
			    SAR_FWRITE(&fp_img,tag+m,sizeof(unsigned __int16));
				SAR_FWRITE(&fp_img,type+m,sizeof(unsigned __int16));
				SAR_FWRITE(&fp_img,len+m,sizeof(unsigned __int32));
				SAR_FWRITE(&fp_img,val_offset+m,sizeof(unsigned __int32));
			   }
			   ////////////////////////IMG DATA FORM////////////////
			   SAR_FWRITE(&fp_img,&idl,sizeof(unsigned __int32));
			   SAR_FWRITE(&fp_img,col,sizeof(unsigned __int16)*3);
			   SAR_FWRITE(&fp_img,res,sizeof(unsigned __int32)*4);
		       SAR_FWRITE(&fp_img,img_data,sizeof(unsigned char)*high*wid);
		 	   SAR_FCLOSE(&fp_img);

			   ///////////////////////////////////////////////////
		
				return 0;
	}

	int ImgMark::Form_RGB8_Tif(unsigned char *img_data,char *img_file,long high,long wid)
    {
		
	  	        FILE        *fp_img;
			   __int64           loc,n,m;
			   unsigned __int16  de_num=15;
			   unsigned __int16  tag[]={254,256,257,258,259,262,273,274,277,278,279,282,283,284,296};
			   unsigned __int16  type[]={4,3,3,3,3,3,4 ,3 ,3,3 ,4 ,5,5,3,3};
			   unsigned __int32  len[]={1,1,1,3,1,1,1,1,1,1,1,1,1,1,1};
			   unsigned __int32  val_offset[]={0,4096,6144,194,1,2,216,1,3,6144,75497472,200,208,1,2};
			   unsigned __int8   ifh[]={73,73,42,0};
			   unsigned __int16  col[]={8,8,8};
			   unsigned __int32  res[]={720000,10000,720000,10000};
			   unsigned __int32   mov=8;
			   unsigned __int32   idl=0;

			   if((high*wid)<10) { printf("图像文件大小不对"); return 0;}
		  
			     val_offset[1]=wid; 
				 val_offset[2]=high;
				 val_offset[9]=high;
				 val_offset[10]=wid*high*3;
			 /////////////////form tiff////////////////////////////////////
			 ////////////IFH FORM///////////////////////
			   SAR_FOPEN(&fp_img,img_file,"wb");
			   SAR_FWRITE(&fp_img,ifh,sizeof(unsigned char)*4);
			   SAR_FWRITE(&fp_img,&mov,sizeof(unsigned __int32));
			   ///////////////////IFD FORM//////////////////
			   SAR_FWRITE(&fp_img,&de_num,sizeof(unsigned __int16));
			   for(m=0;m<de_num;m++)
			   {
			    SAR_FWRITE(&fp_img,tag+m,sizeof(unsigned __int16));
				SAR_FWRITE(&fp_img,type+m,sizeof(unsigned __int16));
				SAR_FWRITE(&fp_img,len+m,sizeof(unsigned __int32));
				SAR_FWRITE(&fp_img,val_offset+m,sizeof(unsigned __int32));
			   }
			   ////////////////////////IMG DATA FORM////////////////
			   SAR_FWRITE(&fp_img,&idl,sizeof(unsigned __int32));
			   SAR_FWRITE(&fp_img,col,sizeof(unsigned __int16)*3);
			   SAR_FWRITE(&fp_img,res,sizeof(unsigned __int32)*4);
		       SAR_FWRITE(&fp_img,img_data,sizeof(unsigned char)*high*wid*3);
		 	   SAR_FCLOSE(&fp_img);

			   ///////////////////////////////////////////////
			   return 0;
   }
	
	template <class T1, class T2>
	void ImgMark::Data2Quan(T1 *pdata, __int64 na, __int64 nr, T2 *pout, char bit, __int64 hist_sum_th)
{
	// 数据智能量化； Dev by Yunhua-Luo @ 2015/5/16 updated @ 2015/5/21; check OK

	// 1. 量化为16bit数据
	__int64 k,j,l;
	long loc=0;
	__int64 M = 10;
	__int64 hist_down = M;
	__int64 hist_up = 65535-M;
	float max_f = max_val(pdata,na*nr,&loc);
	float rat   = 65535.0/max_f;
	
	unsigned __int16 *pt = new unsigned __int16[na*nr];
	memset(pt,0,sizeof(unsigned __int16)*na*nr);
	memset(pout,0,sizeof(T2)*na*nr);
	for(k=0; k<na*nr; k++) pt[k] = MIN(unsigned __int16((pdata[k]*rat)),65535);

	// 2. 求解直方图
	__int64 *hist      = new __int64[65536];
	__int64 *hist_sum1 = new __int64[65536];
	__int64 *hist_sum2 = new __int64[65536];
	memset(hist,0,sizeof(__int64)*65536);
	memset(hist_sum1,0,sizeof(__int64)*65536);
	memset(hist_sum2,0,sizeof(__int64)*65536);
	
	for(k=0; k<na*nr; k++)hist[pt[k]]++;
	hist_sum1[0] = hist[0];
	hist_sum2[65535] = hist[65535];
	for(k=1; k<65536; k++)hist_sum1[k]   = hist_sum1[k-1]+hist[k];
	for(k=65535; k>0; k--)hist_sum2[k-1] = hist_sum2[k]+hist[k-1];

	// 3. 干掉最大和最小部分
	for(k=0; k<65536; k++)
	{
		if(hist_sum1[k] > hist_sum_th){ hist_down = k;break;}
	}
	for(k=65535; k>0; k--)
	{
		if(hist_sum2[k] > hist_sum_th){ hist_up = k;break;}
	}
	printf("hist_sum_th: %d, hist range : ( %d - %d)\n",hist_sum_th,hist_down,hist_up);

	for(k=0; k<na*nr; k++) 
	{
		if(pt[k] > hist_up)   pt[k] = hist_up;
		if(pt[k] < hist_down) pt[k] = hist_down;
	}
	
	// 4. 数据重新量化；
	if(bit > 16) bit = 16;
	__int64 level = __int64(pow(2.0,bit))-1;
	rat = float(level)/(hist_up-hist_down);
	for(k=0; k<na*nr; k++) pout[k] = MIN(T2(rat*(pt[k]-hist_down)),level);

	delete [] pt;
	delete [] hist;
	delete [] hist_sum1;
	delete [] hist_sum2;
}



		// ************ 初始化参数提取的位置 *************

		stParInFrame::stParInFrame()
		{
			startPosion=4*0x60;//偏移量(字节)
			unit=1;//(单位)默认为1
			numType=uInt8Enm; 
		}

		void taskInfoPosition::initial()
		{
			//stParInFrame missionCodes,planeType,planeNum,transType,imgCompRate
			missionCodes.startPosion=4*0x60+51;
			missionCodes.numType=Bcd2Enm;

			planeType.startPosion=4*0x60+53;
			planeType.numType=uInt8Enm;

			planeNum.startPosion=4*0x60+54;
			planeNum.numType=Bcd1Enm;

			//传输类型，图像压缩比
			transType.startPosion=4*0x73;
			transType.numType=uInt8Enm;

			imgCompRate.startPosion=4*0x73+1;
			imgCompRate.numType=uInt16Enm;

			powerOnCnt.startPosion=4*0x11+2;
			powerOnCnt.numType=uInt8Enm;
		}

		void muInfoPosition::initial()
		{
			aAngle.startPosion=4*0x08;
			aAngle.numType=int16Enm;
			aAngle.unit=0.01;

			aAngle_cal.startPosion = 4 * 0x19;
			aAngle_cal.numType = int16Enm;
			aAngle_cal.unit = 0.01;

			// 码盘的转动角
			rAngle.startPosion=4*0x26;
			rAngle.numType=int16Enm;
			rAngle.unit=0.01;

			//距离向预定角
			rsAngle.startPosion = 4 * 0x08 + 2;
			rsAngle.numType = int16Enm;
			rsAngle.unit = 0.01;

			// 距离向的波束运补后的指向角
			rAngle_pt.startPosion = 4 * 0x19 + 2;
			rAngle_pt.numType = int16Enm;
			rAngle_pt.unit = 0.01;

			date_year.startPosion=4*0x60+55;
			date_year.numType=uInt8Enm;

			date_month.startPosion=4*0x60+56;
			date_month.numType=uInt8Enm;

			date_day.startPosion=4*0x60+57;
			date_day.numType=uInt8Enm;

			time_hour.startPosion=4*0x60+58;
			time_hour.numType=uInt8Enm;

			time_minutes.startPosion=4*0x60+59;
			time_minutes.numType=uInt8Enm;

			time_second.startPosion=4*0x60+60;
			time_second.numType=uInt8Enm;

			time_m_second.startPosion=4*0x60+61;
			time_m_second.numType=uInt8Enm;
			time_m_second.unit=5;

			 //经度，纬度，飞机高度，
			plane_longitude.startPosion=4*0x60+23;
			plane_longitude.numType=int32Enm;
			plane_longitude.unit=(1.0/double(2147483648.0))*SC_D; //luo

			plane_latitude.startPosion=4*0x60+27;
			plane_latitude.numType=int32Enm;
			plane_latitude.unit=(1.0/double(2147483648.0))*SC_D; //luo

			plane_height.startPosion=4*0x60+21;
			plane_height.numType=int16Enm;
			plane_height.unit=2.5*FT_M;

			//目标高度
			aim_height.startPosion=4*0x12+2;
			aim_height.numType=int16Enm;
			aim_height.unit=2.5*FT_M;

			//飞机东速度，飞机北速度，飞机天速度，东向加速，北向加速，天向加速
			plane_east_v.startPosion=4*0x60+33;
			plane_east_v.numType=int16Enm;
			plane_east_v.unit=-0.125*FT_M;
			//飞机北速度

			plane_north_v.startPosion=4*0x60+31;
			plane_north_v.numType=int16Enm;
			plane_north_v.unit=0.125*FT_M;
			//飞机天速度
			plane_up_v.startPosion=4*0x60+35;
			plane_up_v.numType=int16Enm;
			plane_up_v.unit=0.125*FT_M;
			//东向加速
			plane_east_a.startPosion=4*0x60+39;
			plane_east_a.numType=int16Enm;
			plane_east_a.unit=-0.015625*FT_M;
			//北向加速
			plane_north_a.startPosion=4*0x60+37;
			plane_north_a.numType=int16Enm;
			plane_north_a.unit=0.015625*FT_M;

			//天向加速
			plane_up_a.startPosion=4*0x60+41;
			plane_up_a.numType=int16Enm;
			plane_up_a.unit=0.015625*FT_M;


			//飞机航向角 飞机横滚角 飞机俯仰角 飞机偏航角
			plane_direction_angle.startPosion=4*0x60+43;
			plane_direction_angle.numType=int16Enm;
			plane_direction_angle.unit=(1.00/double(32768.00))*SC_D;

			//飞机横滚角
			plane_hor_angle.startPosion=4*0x60+45;
			plane_hor_angle.numType=int16Enm;
			plane_hor_angle.unit=(1.00/32768.00)*SC_D;

			//飞机俯仰角
			plane_dive_angle.startPosion=4*0x60+47;
			plane_dive_angle.numType=int16Enm;
			plane_dive_angle.unit=(1.00/double(32768.00))*SC_D;

			//飞机偏航角
			plane_departure_angle.startPosion=4*0x60+49;
			plane_departure_angle.numType=int16Enm;
			plane_departure_angle.unit=(1.00/double(32768.00))*SC_D;

			plane_ground_v.startPosion=4*0x71;
			plane_ground_v.numType=uInt32Enm;
			plane_ground_v.unit=0.001;

			//scanCode,date_year,date_month,date_day,time_hour,time_minutes,time_second,time_m_second,
			//飞机目标相对高度，飞机俯仰角，俯仰角速度，俯仰角加速度
			//plane_dive_angle_V,
			//飞机横滚角，横滚速度，横滚加速度，
			//plane_dive_angle_a,plane_hor_angle,plane_hor_angle_v,plane_hor_angle_a,
			//飞机偏流角，飞机偏航角速度，飞机偏航角加速度，地速，真空速度，指示速度
			//plane_de_flow_angle,plane_departure_v,plane_departure_a,plane_ground_v,plane_noair_v,plane_point_v,
	
		}

		void sarImageInfoPosition::initial()
		{
			loop_num.startPosion=4*0x72; // modified by luo remove +2 
			loop_num.numType=uInt16Enm;

			band.startPosion=4*0x0A;
			band.numType=uInt32Enm;

			img_center_height.startPosion=4*0x12;
			img_center_height.numType=int16Enm;
			img_center_height.unit=2.5*FT_M;
			//sAngle.startPosion=4*0x08;
			work_mode.startPosion=4*0x04+1;

			//距离向预定角
			rsAngle.startPosion=4*0x08+2;
			rsAngle.numType=int16Enm;
			rsAngle.unit=0.01;

			// 惯导计算后的角度
			rAngle.startPosion=4*0x26;
			rAngle.numType=int16Enm;
			rAngle.unit=0.01;

			// 实际的波束指向角
			rAngle_pt.startPosion = 4*0x19+2;
			rAngle_pt.numType=int16Enm;
			rAngle_pt.unit=0.01;

			//方位向预定角
			beam_FW_Angle.startPosion=4*0x08;
			beam_FW_Angle.numType=int16Enm;
			beam_FW_Angle.unit=0.01;
			
			//分辨率
			sarRes.startPosion=4*0x0F;
			sarRes.numType=uInt32Enm;
			
			dopler_center.startPosion=4*0x70;
			dopler_center.numType=int32Enm;

			//波束水平宽度
			beam_horz_width.startPosion=4*0x10;
			beam_horz_width.numType=uInt32Enm;

			prf.startPosion=4*0x09;
			prf.numType=uInt32Enm;
			
		    prfCount.startPosion=4;
			prfCount.numType=uInt32Enm;
			prfCount.unit=1;

			//采样起始 脉冲宽度 
			sampleStart.startPosion=0x05*4;
			sampleStart.unit=(10.0E-9);
			sampleStart.numType=uInt32Enm;
			
			Tp.startPosion=0x0C*4;
			Tp.unit=(10.0E-9);
			Tp.numType=uInt32Enm;

			//采样点数
			samples.startPosion=0x0B*4;
			samples.numType=uInt32Enm;
			//
			a_point_size.startPosion=4*0x71;
			a_point_size.numType=uInt32Enm;

			////飞机高度，为计算场景中心高度
			//scene_center_height.startPosion=4*0x60+21;
			//scene_center_height.numType=int16Enm;
			//scene_center_height.unit=2.5*FT_M;

			plane_height.startPosion=4*0x60+21;
			plane_height.numType=int16Enm;
			plane_height.unit=2.5*FT_M;

			//目标高度
			aim_height.startPosion=4*0x12+2;
			aim_height.numType=int16Enm;
			aim_height.unit=2.5*FT_M;

			//采样起始_最近斜距
			slope_nearest.startPosion=4*0x05;
			slope_nearest.numType=uInt32Enm;
			slope_nearest.unit=1.5;  // 10e-9*1.5e8; 

			sampoints.startPosion=4*0x0B;
			sampoints.numType=uInt32Enm;

			//工作模式
			work_mode.startPosion=4*0x04;
			work_mode.numType=uInt32Enm;

			multipleView.startPosion=4*0x22;
			multipleView.numType=uInt32Enm;

			//经度，纬度，飞机高度，
			plane_longitude.startPosion=4*0x60+23;
			plane_longitude.numType=int32Enm;
			plane_longitude.unit=(1.0/double(2147483648.0))*SC_D; //luo

			plane_latitude.startPosion=4*0x60+27;
			plane_latitude.numType=int32Enm;
			plane_latitude.unit=(1.0/double(2147483648.0))*SC_D; //luo
		}

		void sarGMTIAimInfoPosition::initial()
		{
			//目标个数
			aimsNum.startPosion=512;
			aimsNum.numType=uInt16Enm;
			//一景图像方位向点数
			scAzPoints.startPosion=514;
			scAzPoints.numType=uInt16Enm;

			//aimRangePoints.startPosion=1024;
			//aimRangePoints.numType=uInt16Enm;

			//aimAzimuthPoints.startPosion=1026;
			//aimAzimuthPoints.numType=int32Enm;
			//	
			//aimV.startPosion=1030;
			//aimV.numType=int16Enm;
			//aimV.unit=1.0/256.0;

			//aimStrength.startPosion=1032;
			//aimStrength.numType=uInt16Enm;
		}
		

		void GYAimInfoPosition::initial()
		{
			aimsNum.startPosion=512;
			aimsNum.numType=uInt16Enm;
			//aimsNum.numType=uInt32Enm;

			////驻留脉冲数
			//pulseResident.startPosion=0x04*4;
			//pulseResident.numType=uInt16Enm;
		}
				
		void NewGYAimInfoPosition::initial()
		{
			aimsNum.startPosion=512;		// modified by Luo 202003/04 本包的目标数
			aimsNum.numType=uInt16Enm;
			//aimsNum.numType=uInt32Enm;

			aimsAll.startPosion=512+2;		// modified by Luo 202003/04 本包的目标数
			aimsAll.numType=uInt16Enm;
			
			detect_th.startPosion=4*0x12+2;
			detect_th.numType=int16Enm;

			////驻留脉冲数
			pulseResident.startPosion=0x04*4;
			pulseResident.numType=uInt16Enm;

		}
		
		void GYParsInfoPosition::initial()
		{

			/*frameCodes,waveCodes,scanCenterAngle,scanStep,pulseResident,powerOnTimes,workModeFr,loadCode,loadType,
		         resvAb,spotWid,prf;*/

			//帧编号
			frameCodes.startPosion=0x2a*4+1;
			frameCodes.numType=uInt8Enm;
			//波位号
			waveCodes.startPosion=0x2a*4;
			waveCodes.numType=uInt8Enm;

			//中心扫描角--固定值
			scanCenterAngle.startPosion=0x2b*4+2;
			scanCenterAngle.numType=int16Enm;
			scanCenterAngle.unit=0.01;

			//扫描范围
			scanScope.startPosion=0x2b*4;
			scanScope.numType=int16Enm;
			scanScope.unit=0.01;
		
			scanAngle.startPosion=4*0x08;
			scanAngle.numType=int16Enm;
			scanAngle.unit=0.01;
			//驻留脉冲数
			pulseResident.startPosion=0x04*4;
			pulseResident.numType=uInt16Enm;
			//带宽
			//10 对应50
			//20 对应25
			band.startPosion=4*0x0A;
			band.numType=uInt32Enm;
			//工作模式
			workMode.startPosion=0x04*4;
			workMode.numType=uInt32Enm;

			prf.startPosion=4*0x09;
			prf.numType=uInt32Enm;
			//采样起始 脉冲宽度 
			sampleStart.startPosion=0x05*4;
			sampleStart.unit=(10.0E-9);
			sampleStart.numType=uInt32Enm;
			//
			Tp.startPosion=0x0C*4;
			Tp.unit=(10.0E-9);
			Tp.numType=uInt32Enm;
			//采样点数
			samples.startPosion=0x0B*4;
			samples.numType=uInt32Enm;
			//方位波束预定角
			//azimuthCenterAngle.startPosion=0x19*4+2;
			azimuthCenterAngle.startPosion=0x08*4;
			azimuthCenterAngle.numType=int16Enm;
			azimuthCenterAngle.unit=0.01;

			powerOnTimes.startPosion=4*0x11+2;
			powerOnTimes.numType=uInt8Enm;

			////目标高度
			//aim_height.startPosion=4*0x12;
			//aim_height.numType=int16Enm;
			//aim_height.unit=2.5*FT_M;
			////飞机高度
			//plane_height.startPosion=4*0x60+21;
			//plane_height.numType=int16Enm;
			//plane_height.unit=2.5*FT_M;

			////采样起始_最近斜距
			//slope_nearest.startPosion=4*0x05;
			//slope_nearest.numType=uInt32Enm;
			//slope_nearest.unit=1.5;

		}


		// ******* 飞行任务参数获取 **********

		taskInfo::taskInfo(UINT8 *ar)
		{
			taskInfoPosition ps;
			ps.initial();

			fmtConvCl fmtConv(ar);

			missionCodes=fmtConv.getResult(ps.missionCodes);

			planeType=fmtConv.getResult(ps.planeType);

			planeNum=fmtConv.getResult(ps.planeNum);

			transType=fmtConv.getResult(ps.transType);

			double tTypeU=transType;
			//时传经常出现数值很大的SB错误,所以弄个补丁
			if(tTypeU>2)
			{ 
				transType=1;
			}
			
			imgCompRate=fmtConv.getResult(ps.imgCompRate);
			powerOnCnt=fmtConv.getResult(ps.powerOnCnt);

		}
		
		int taskInfo::output(FILE * FL)
		{	
			fprintf(FL,"任务代号:%04d\n",long(missionCodes));
			fprintf(FL,"飞行器类型:07\n");
			fprintf(FL,"飞行批号:%02d\n",int(planeType));
			fprintf(FL,"飞机号:%04d\n",long(planeNum));
			fprintf(FL,"传输方式(0=地面回读,1=实时传输,2=选择传输,3=精细成像):%01.0f\n",transType);
			fprintf(FL,"图像压缩比:%03d\n",int(imgCompRate));
			fprintf(FL,"载荷类型(01-高空CCD,02-长焦倾斜CCD,03-SAR,04-红外行扫仪,05-多光谱):03\n");
			fprintf(FL,"载荷编号:000000\n");
			fprintf(FL,"开机次数:%03.0f\n",powerOnCnt);

			return 0;
		}
		
		taskInfo::~taskInfo()
		{
		}
		

		//********** 惯导参数获取 ****************

		muInfo::muInfo(UINT8 *ar)
		{
			muInfoPosition ps;
			ps.initial();

			fmtConvCl fmtConv(ar);
			
			aAngle=fmtConv.getResult(ps.aAngle);

			azAngle_cal = fmtConv.getResult(ps.aAngle_cal);

			rAngle=fmtConv.getResult(ps.rAngle);

			rsAngle = fmtConv.getResult(ps.rsAngle);

			rAngle_pt = fmtConv.getResult(ps.rAngle_pt);

			date_year=fmtConv.getResult(ps.date_year);

			date_year = long(date_year)%2000+2000;

			date_month=fmtConv.getResult(ps.date_month);

			date_day=fmtConv.getResult(ps.date_day);

			time_hour=fmtConv.getResult(ps.time_hour);

			time_minutes=fmtConv.getResult(ps.time_minutes);
	
			time_second=fmtConv.getResult(ps.time_second);
		
			time_m_second=fmtConv.getResult(ps.time_m_second);

			// modified by luo 

			time_second = time_second + int(time_m_second/1000);

			time_minutes = time_minutes + int(time_second/60);

			time_second = long(time_second) % 60;

			time_m_second = long(time_m_second) % 1000;


			//高度,经度，纬度，
			plane_height=fmtConv.getResult(ps.plane_height);
	
			aimH = fmtConv.getResult(ps.aim_height);

			int temph = int(aimH+0.5);   // 如何处理

			cfar_th_dB = (temph - int(temph / 100.0) * 100) & 63;
			if (cfar_th_dB == 0)
				cfar_th_dB = 18.0;

			plane_aim_height=plane_height-aimH;

			plane_longitude=fmtConv.getResult(ps.plane_longitude); // start of burst
			//纬度
			plane_latitude=fmtConv.getResult(ps.plane_latitude);

			//飞机东速度，飞机北速度，飞机天速度，东向加速，北向加速，天向加速
			plane_east_v=fmtConv.getResult(ps.plane_east_v);

			//飞机北速度
			plane_north_v=fmtConv.getResult(ps.plane_north_v);
			//飞机天速度
			plane_up_v=fmtConv.getResult(ps.plane_up_v);
			//东向加速
			plane_east_a=fmtConv.getResult(ps.plane_east_a);

			plane_north_a=fmtConv.getResult(ps.plane_north_a);
			//天向加速
			plane_up_a=fmtConv.getResult(ps.plane_up_a);

			//飞机航向角 飞机横滚角 飞机俯仰角 飞机偏航角
			plane_direction_angle = fmtConv.getResult(ps.plane_direction_angle);
			if (plane_direction_angle < 0) plane_direction_angle += 360;

			// 偏流角的计算
			float track_angle = atan2(plane_east_v, plane_north_v)*180.0/PI;
			if (track_angle < 0) track_angle += 360.0;
			plane_de_flow_angle =  track_angle - plane_direction_angle; // 

			plane_hor_angle=fmtConv.getResult(ps.plane_hor_angle);

			plane_dive_angle=fmtConv.getResult(ps.plane_dive_angle);

			plane_departure_angle=fmtConv.getResult(ps.plane_departure_angle);

			plane_ground_v=sqrt(plane_east_v*plane_east_v+plane_north_v*plane_north_v);
			
		}
		
		muInfo::~muInfo()
		{
		}
		
		int muInfo::output(FILE * FL,int gdNum)
		{
			    fprintf(FL,"\n惯导序号:%05d\n",gdNum);  // modify by zhang
			    fprintf(FL,"日期(年.月.日):%04.0f.%02.0f.%02.0f\n",date_year,date_month,date_day);
				fprintf(FL,"时间(时-分-秒-毫秒):%02.0f-%02.0f-%02.0f-%03.0f\n",time_hour,time_minutes,time_second,time_m_second);
				fprintf(FL,"飞机经度(度):%.7f\n",plane_longitude);
				fprintf(FL,"飞机纬度(度):%.7f\n",plane_latitude);
				fprintf(FL,"飞机海拔高度(米):%.3f\n",plane_height);
				fprintf(FL,"机下点海拔高度(米):%.3f\n\n",aimH);
				
				fprintf(FL, "俯仰波束预设值(度):%.3f\n", rsAngle);
				fprintf(FL, "俯仰波束码盘值(度):%.3f\n", rAngle);
				fprintf(FL, "俯仰波束波控计算值(度):%.3f\n", rAngle_pt);
				
				fprintf(FL,"飞机航向角(度):%.7f\n",plane_direction_angle);
				fprintf(FL,"飞机航向角速度(度/秒):0.000\n");
				fprintf(FL,"飞机航向角加速度(度/平方秒):0.000\n");

				fprintf(FL,"飞机俯仰角(度):%.7f\n",plane_dive_angle);
				fprintf(FL,"飞机俯仰角速度(度/秒):0.000\n");
				fprintf(FL,"飞机俯仰角加速度(度/平方秒)0.000\n");

				fprintf(FL,"飞机横滚角(度):%.7f\n",plane_hor_angle);
				fprintf(FL,"飞机横滚角速度(度/秒):0.000\n");
				fprintf(FL,"飞机横滚角加速度(度/平方秒):0.000\n");

				fprintf(FL,"飞机偏航角(度):%.7f\n",plane_departure_angle);
				fprintf(FL,"飞机偏流角(度):%.7f\n", plane_de_flow_angle);

				fprintf(FL,"地速(米/秒):%.4f\n",plane_ground_v);
				fprintf(FL,"真空速(米/秒):%.4f\n",plane_ground_v);
				fprintf(FL,"指示空速(米/秒):%.4f\n",plane_ground_v);

				fprintf(FL,"飞机东速(米/秒):%.4f\n",plane_east_v);
				fprintf(FL,"飞机北速(米/秒):%.4f\n",plane_north_v);
				fprintf(FL,"飞机天速(米/秒):%.4f\n",plane_up_v);
				fprintf(FL,"东向加速度(米/平方秒):%.4f\n",plane_east_a);
				fprintf(FL,"北向加速度(米/平方秒):%.4f\n",plane_north_a);
				fprintf(FL,"天向加速度(米/平方秒):%.4f\n",plane_up_a);

				return 0;
		}
		
		 
		//******** 图像和载荷参数获取 *************

		sarImageInfo::sarImageInfo(UINT8 *ar)
		{// 2020/03/05 check up

				sarImageInfoPosition sarImagePs;
			    sarImagePs.initial();
				
				muInfoPosition muPs;
				muPs.initial();
				muInfo muPars(ar);

				year  = muPars.date_year;
				month = muPars.date_month;
				day   = muPars.date_day;
	
				GMTI_OFFSET_ERR = false;
				
				fmtConvCl fmtConv(ar);

				strip_num=1;
			
				commonFuncs cmFuncs;
				img_cols=(double)cmFuncs.getRangePoints(ar);
				int mdTempp = cmFuncs.getSarModel(ar);  // 工作模式
				sar_mode = mdTempp;

				//图像行数，固定值；
				img_rows=512;
				img_deepth=8.0;
				
				load_code=0x67;
				load_type=3;

				UINT32 bmHorWdTemp1=(UINT32)fmtConv.getResult(sarImagePs.beam_horz_width);
				int bmHorWdTemp2=(bmHorWdTemp1>>24)&0x0F;
				if(bmHorWdTemp2>-1 && 5>bmHorWdTemp2)
				{
				    beam_horz_width=BEAM_WIDTH[bmHorWdTemp2-1];
				}
				else
				{
                   beam_horz_width=BEAM_WIDTH[0];
				}

				//prf值
				double dbTmp=fmtConv.getResult(sarImagePs.prf);
				if(dbTmp!=0.0)
				{
					prf=1.0E8/dbTmp;
				}
				else
				{
				    prf=ERROR_Value;
				}

				loop_num = cmFuncs.getLoopNum(ar);

				prfCount=fmtConv.getResult(sarImagePs.prfCount);
				
				//乘以prf,除以2^29
				// dopler_center=prf*(fmtConv.getResult(sarImagePs.dopler_center))/(536870912.0);  // should be updated ???
				dopler_center = cmFuncs.getDopplerCenter(ar,prf);
				
				//距离向预定角
				rsAngle = muPars.rsAngle; // fmtConv.getResult(sarImagePs.rsAngle);

				//azAngle = muPars.aAngle; 
				azAngle = muPars.aAngle;

				azAngle_cal = muPars.azAngle_cal;
				
				// 惯导计算后的角度
				rAngle = muPars.rAngle; // fmtConv.getResult(sarImagePs.rAngle);

				beam_FW_Angle=fmtConv.getResult(sarImagePs.beam_FW_Angle);
				//分辨率
				UINT32 sarResIndex1=(UINT32)fmtConv.getResult(sarImagePs.sarRes);

				int sarResIndex2=sarResIndex1>>25 & 0x07;

                if((sarResIndex2 >= 0) && (sarResIndex2 < 8))  // modify by zhang : if(0<=sarResIndex2<8)
				{
					sarRes=RS_ARRAY[sarResIndex2];
				}
				else
				{
					sarRes=3; //ERROR_Value; // modified by luo
				}
				 
				load_code=0;
				//固定值：03
				load_type=3;

				//带宽 采样频率
				int temp=0;
				temp=(int)fmtConv.getResult(sarImagePs.band);				
				temp=(temp >> 21) & 0x0F;
				if((temp >= 0) && (temp < 5))  // modify by zhang : if(0<=temp<5)
				{
					band=(double)BD_ARRAY[temp];
					fs=(double)FS_ARRAY[temp];
				}
				else  // 二次判断参数
				{
					// ---- judge ----
					//RS_ARRAY[]={0.150,0.30,0.50,1.0,3.0,5.0,10.0,20.0};
					float FS_MORE[] = {1600,800,500,250,125,62.5,31.25,31.25};
					float BD_MORE[] = {1360,700,420,210,80,80,25,25};

					if((sarResIndex2 >= 0) && (sarResIndex2 < 8))
					{
						band = BD_MORE[sarResIndex2];
						fs = FS_MORE[sarResIndex2];
						if(mdTempp==4 && sarResIndex2==6)
						{
							fs = 125;
							band = 80;
						}
					}else
					{
						band = 210;
						fs = 250;
					}
				}
				
				//斜距分辨率
				slope_res=C_LIGHT/(2*band*1.0e6);	
				////方位向像元尺寸所用地速
				double	dbplane_east_v=fmtConv.getResult(muPs.plane_east_v);
				double dbplane_north_v=fmtConv.getResult(muPs.plane_north_v);
				double dbplane_up_v = fmtConv.getResult(muPs.plane_up_v);
				double az_vel = sqrt(dbplane_east_v*dbplane_east_v+dbplane_north_v*dbplane_north_v);
		    
				//方位向像元尺寸
				if(prf!=0) 
					a_point_size = az_vel/prf;
				else
					a_point_size=ERROR_Value;

				//距离向像元尺寸 11/2 加系数1.1
				if(fs!=0)
				   r_point_size = C_LIGHT/(2*fs*1.0e6);
				else
				   r_point_size=ERROR_Value;

				//地距分辨率
				double ground_res_temp=abs(sin(rsAngle*PI/180));

				if(ground_res_temp!=0)
				   ground_res=slope_res/ground_res_temp;
				else
					ground_res=ERROR_Value;

				double sampleStart0 = fmtConv.getResult(sarImagePs.sampleStart);
				plane_height  = fmtConv.getResult(sarImagePs.plane_height);
				aim_height    = fmtConv.getResult(sarImagePs.aim_height);
				Nprf = ar[28]& 0x03;
				sampleStart = sampleStart0+Nprf/prf; ;
		
				if(mdTempp<4) rangePoints = cmFuncs.getRangePoints(ar);
				Tp=fmtConv.getResult(sarImagePs.Tp);
				samples=fmtConv.getResult(sarImagePs.samples);
				sampoints=fmtConv.getResult(sarImagePs.sampoints);
				slope_nearest=(sampleStart-Tp/2)*C_LIGHT/2;

				if(mdTempp == 4)
				{
					rangePoints = sampoints-Tp*fs*1.0e6;
					range_offset = r_point_size*(Tp/2*fs*1.0e6);	// added by Luo 20190615--修正WAS-MTI的距离向偏移

				}else
				{
					range_offset = (sampoints-rangePoints)/2*r_point_size;
					if(mdTempp == 3 && sampoints < 65536)
					{
						range_offset = long(Tp*fs*1.0e6/2/512)*512*r_point_size;
					}
				}
				slope_nearest = slope_nearest+range_offset;

				if(rangePoints>0&&fs!=0)
				{
					slope_far=slope_nearest+rangePoints*r_point_size;
					slope_sen_center=slope_nearest+rangePoints/2*r_point_size; // 图像中心斜距
				}
				else
				{
					slope_sen_center=slope_far=slope_nearest;					
				}

				double	elevationAngle=fmtConv.getResult(muPs.rAngle)*PI/180;

				if(elevationAngle<0) //right side 
				{
					look_Side=-1;
				}
				else				//left side 
				{
					look_Side=1;
				}

				UINT32 wkMd=(UINT32)fmtConv.getResult(sarImagePs.work_mode);
				
				work_mode=(wkMd>>16) & 0x0F;
				//1：0.3M；2：0.5M；3：1M；4：3M；5：5M；6：10M；7：20M；
				//工作模式(01=条带0.5m,02=条带1m,03=条带3m,04=聚束0.3m,05=SAR/GMTI3m,06=SAR/GMTI5m,07=SAR/GMTI10m,08=广域GMTI,09=广域MMTI,10=0.15m条带 11=0.3m条带)
				switch (sarResIndex2)
				{
				case 0:
					work_mode_num=10;//0.15米条带
					break;

				case 1:
					if(work_mode==0)
						work_mode_num=11;	//0.3米条带
					else
						work_mode_num=4;	//0.3米聚束
					break;

				case 2:
					work_mode_num=1;//0.5米条带
					break;

				case 3:
					work_mode_num=2;//1米条带
					break;

				case 4:            //3米
					if((int)work_mode==0)
					{
						work_mode_num=3;
					}
					else
					{
						work_mode_num=5;
					}
					break;
				case 5:          //5米
					work_mode_num=6;
					break;
				case 6:
					work_mode_num=7;
					break;  // add by zhang
				case 7:
					work_mode_num=8;
				}

				if((int)work_mode==4)
				{
					work_mode_num=8;
				}

				// 多视参数读取 		
				multipleView = cmFuncs.getMViewNum(ar);

				ml2_coef = 1.0;
				if(ACCURATE_ML && sar_mode <= 3) // SAR OR SAR-GMTI Mode
				{
					ml2_coef = float(a_point_size*multipleView)/r_point_size;
				}

				img_width  = r_point_size*img_cols;//图像幅宽
				img_length = TIFHEIGHT*a_point_size*multipleView; //图像长度


				double plane_height1 = plane_height-aim_height;
				ground_nearest = sqrt(slope_nearest*slope_nearest-plane_height1*plane_height1);  // luo
				ground_far     = sqrt(slope_far*slope_far-plane_height1*plane_height1);  // luo

				syApTime = beam_horz_width*PI*slope_far/(180*az_vel);
				plane_longitude = fmtConv.getResult(sarImagePs.plane_longitude);
				plane_latitude  = fmtConv.getResult(sarImagePs.plane_latitude);

				plane_east_v    = dbplane_east_v;
				plane_north_v   = dbplane_north_v;
				plane_up_v = dbplane_up_v;
				
				scene_center_height  =  plane_height-aim_height;
				double a_Angle_far  = 180/PI*acos(scene_center_height/slope_far);
				double a_Angle_near = 180/PI*acos(scene_center_height/slope_nearest);

				beam_R_width = abs(a_Angle_far-a_Angle_near);
				beam_A_width = beam_horz_width;

				//a_angle_new = ((int)(fmtConv.getResult(muPs.plane_direction_angle)-fmtConv.getResult(muPs.plane_departure_angle)-look_Side*90+360)%360); // 方位波束中心与正北夹角
				
				R_angle_new = 90-abs(rAngle); // by luo 

		}	
		
		void sarImageInfo::sarImageLocating(double pre_lati,double pre_logn,double cur_lati, double cur_logn)
		{// checked by luo 20121/07/12 modified by 
				
				double lati_s  = pre_lati; // 图像方位起始和结束位置飞机的经度
				double logn_s = pre_logn;
			
				double lati_e  = cur_lati; // 图像方位起始和结束位置飞机的纬度
				double logn_e = cur_logn; 
				
				float  look_side = look_Side; // 侧视方向，左侧 1， 右侧为-1；
				double fdc = 0;						//dopler_center; 2018/11/15 fixed with real time processing.
				double lamda = 0.03125;

				double samps = sampoints;
				double dr = r_point_size;
				double offset,r_near,r_far;

				//aim_height = 131;
				r_near = slope_nearest;
				r_far  = slope_far;
		
				double vn_s,vn_e,ve_s,ve_e,vu_s,vu_e;
				vn_s = vn_e = plane_north_v; 
				ve_s = ve_e = plane_east_v;
				vu_s = vu_e = 0.0;

				///--- rd-locating ----
				double latis[4]  = {lati_s,lati_s,lati_e,lati_e};
				double logns[4]  = {logn_s,logn_s,logn_e,logn_e};
				double imu_vn[4] = {vn_s,vn_s,vn_e,vn_e};
				double imu_ve[4] = {ve_s,ve_s,ve_e,ve_e};
				double imu_vu[4] = {vu_s,vu_s,vu_e,vu_e};
				double r0s[4]	 = {r_near,r_far,r_near,r_far};
				double latlogn_rd[2],latlogn_geo[2];
				double lati_r[4];
				double logn_r[4];
				int k,j;

				Tarlocate rd_loc;
				rd_par_input par_loc;
				for(k=0; k<4; k++)
				{
					par_loc.h_gnd = aim_height;
					par_loc.lambda = lamda;
					par_loc.href = plane_height;
					par_loc.lookside = look_side;

					par_loc.fdc = fdc;
					par_loc.lati = latis[k];
					par_loc.logn = logns[k];
					par_loc.r0 = r0s[k]; // improve factor
					par_loc.ve = imu_ve[k];
					par_loc.vn = imu_vn[k];
					par_loc.vu = 0;//imu_vu[k];

					rd_loc.RD_locating(par_loc,latlogn_rd,latlogn_geo);

					lati_r[k] = latlogn_rd[0];
					logn_r[k] = latlogn_rd[1];

				}

				// add gps information for target location 
				gps_info[0] = img_rows; gps_info[1] = img_cols;
				gps_info[2] = lati_r[0]; gps_info[3] = lati_r[1]; gps_info[4] = lati_r[2]; gps_info[5] = lati_r[3]; 
				gps_info[6] = logn_r[0]; gps_info[7] = logn_r[1]; gps_info[8] = logn_r[2]; gps_info[9] = logn_r[3]; 
				
				// output results 
				if(look_Side==-1)	//right side 
				{
				
					left_top_longitude   = logn_r[2];
					left_top_latitude    = lati_r[2];

					left_down_longitude  = logn_r[0];
					left_down_latitude   = lati_r[0];

					right_up_longitude   = logn_r[3];
					right_up_latitude    = lati_r[3];

					right_down_longitude = logn_r[1];
					right_down_latitude  = lati_r[1];

				}else			// image reverse
				{
					left_top_longitude   = logn_r[3];
					left_top_latitude    = lati_r[3];

					left_down_longitude  = logn_r[1];
					left_down_latitude   = lati_r[1];

					right_up_longitude   = logn_r[2];
					right_up_latitude    = lati_r[2];

					right_down_longitude = logn_r[0];
					right_down_latitude  = lati_r[0];

				}

				img_center_longitude = (left_top_longitude+right_down_longitude)/2;
				img_center_latitude  = (left_top_latitude+right_down_latitude)/2;

		}

		int sarImageInfo::output(FILE *FL)
		{
			static int cntRecvpLoop=0;
			static int  numToMerge=8;			
			static double par[30];

			//fprintf(FL,"[飞行任务参数列表%:]\n");
			fprintf(FL,"周期号:%05.0f\n",loop_num);
			fprintf(FL,"条带号: 01\n");

			if(ml2_coef < 1) 
			{
				fprintf(FL,"图像列数(列):%.0f\n",img_cols);
				fprintf(FL,"图像行数(行):%.0f\n",double(int(img_rows*ml2_coef/4.0)*4));
			}else
			{
				fprintf(FL,"图像列数(列):%.0f\n",double(int(img_cols/ml2_coef/4.0)*4));
				fprintf(FL,"图像行数(行):%.0f\n",img_rows);
			}
			fprintf(FL,"图像位深(位): %02.0f\n",img_deepth);
			fprintf(FL,"图像中心点经度(度):%.7f\n",img_center_longitude);
			fprintf(FL,"图像中心点纬度(度):%.7f\n",img_center_latitude);
			fprintf(FL,"图像中心点高度(米):%.3f\n",aim_height);

			fprintf(FL,"左上角经度(度):%6.7f\n",left_top_longitude);
			fprintf(FL,"左上角纬度(度):%6.7f\n",left_top_latitude);

			fprintf(FL,"左下角经度(度):%6.7f\n",left_down_longitude);
			fprintf(FL,"左下角纬度(度):%6.7f\n",left_down_latitude);

			fprintf(FL,"右上角经度(度):%6.7f\n",right_up_longitude);
			fprintf(FL,"右上角纬度(度):%6.7f\n",right_up_latitude);

			fprintf(FL,"右下角经度(度):%6.7f\n",right_down_longitude);
			fprintf(FL,"右下角纬度(度):%6.7f\n",right_down_latitude);

			int wkMode=(int)work_mode;
            if((wkMode >= 0) && (wkMode <= 4))		// modify by zhang : if(0<=wkMode<4)
			{
				fprintf(FL,"工作模式(01=条带0.5m,02=条带1m,03=条带3m,04=聚束0.3m,05=SAR/GMTI3m,06=SAR/GMTI5m,07=SAR/GMTI10m,08=广域GMTI,09=广域MMTI,10=0.15m条带,11=0.3m条带):%02d\n",int(work_mode_num));
			}
			else
			{
				fprintf(FL,"工作模式(01=条带0.5m,02=条带1m,03=条带3m,04=聚束0.3m,05=SAR/GMTI3m,06=SAR/GMTI5m,07=SAR/GMTI10m,08=广域GMTI,09=广域MMTI,10=0.15m条带,11=0.3m条带):模式异常\n");
			}

			fprintf(FL,"工作频段(0=X,1=L,2=P,3=P+L,4=Ku):0\n");

			if(rsAngle>0)
			{
				fprintf(FL,"侧视方式(0=左侧视,1=右侧视):0\n");  // modify by zhang
			}
			else
			{
				fprintf(FL,"侧视方式(0=左侧视,1=右侧视):1\n");  // modify by zhang
			}

			if(ml2_coef < 1) 
			{
				fprintf(FL,"方位向像元尺寸(米):%02.5f\n",a_point_size*multipleView/ml2_coef);
				fprintf(FL,"距离向像元尺寸(米):%02.5f\n",r_point_size);
			}else
			{
				fprintf(FL,"方位向像元尺寸(米):%02.5f\n",a_point_size*multipleView);
				fprintf(FL,"距离向像元尺寸(米):%02.5f\n",r_point_size*ml2_coef);
			}

			fprintf(FL,"地距分辨率(米):%02.2f\n",ground_res);

			fprintf(FL,"斜距分辨率(米):%02.2f\n",slope_res);
			
			//fprintf(FL,"图像幅宽(米):%06.2f\n",img_width);
			fprintf(FL, "图像幅宽(米):%06.2f\n", ground_far- ground_nearest);
			fprintf(FL,"图像长(米):%06.2f\n",img_length);		

			fprintf(FL,"最近斜距(米):%.2f\n",slope_nearest);
			fprintf(FL,"最远斜距(米):%.2f\n",slope_far);
			fprintf(FL,"场景中心斜距(米):%.2f\n",slope_sen_center);

			fprintf(FL,"最近地距(米):%.2f\n",ground_nearest);
			fprintf(FL,"最远地距(米):%.2f\n",ground_far);

			fprintf(FL,"场景中心高度(米):%.2f\n",scene_center_height);
			if(ml2_coef < 1) 
			{
				fprintf(FL,"视数:%.1f\n",multipleView/ml2_coef);
			}else
			{
				fprintf(FL,"视数:%.1f\n",multipleView);
			}
			fprintf(FL,"合成孔径时间(秒):%.3f\n",syApTime);
			fprintf(FL,"重复频率(Hz):%04.0f\n",prf);
			fprintf(FL,"多普勒中心频率(HZ):%04.0f\n",dopler_center);
			fprintf(FL,"波束方位向宽度(度):%.2f\n",beam_horz_width);
			fprintf(FL,"波束俯仰向宽度(度):%.2f\n",beam_R_width);	
		    fprintf(FL,"波束方位角(度):%.4f\n",azAngle);
			fprintf(FL,"波束俯仰角(度):%.4f\n",R_angle_new);
			fprintf(FL,"波束水平宽度(度):%.2f\n",beam_horz_width);
			fprintf(FL,"成像时间(秒): %.3f\n",6+rand()/100.0);
			int N = 2; 
			if(sarRes == 0.15 || sarRes == 3) N = 4;
			fprintf(FL,"数据量(兆比特/秒): %.0f\n",32768.0*prf/N*1.5/1024/1024);
			fprintf(FL,"重要目标标识(0-不重要目标,1-重要目标,2-未知):2\n");
			fprintf(FL,"PRF计数:%.0f\n",prfCount);
			fprintf(FL, "波束运补方位角(度):%.4f\n", azAngle_cal);

			return 0;

		}


		
		// SAR/GMTI定位参数求解
        sarGMTIAimInfo::sarGMTIAimInfo(UINT8* ar, UINT8* arImage, double *gps_info, string tar_map_file)
		{// check by Zhang and luo 2016/7/14 OK  
         // for merged image
			// ar -- track aux data
			// arImage -- last small image aux data;
			
			aimRangePoints=NULL;
			aimAzimuthPoints=NULL;
			Tar_Azloc=NULL;
			Tar_azrev=NULL;
			aimV=NULL;
			aimStrength=NULL;
			aimLongitude=NULL;
			aimlatitude=NULL;

			commonFuncs cmFuncs;

			//用于标识初始化是否成功
			blInitial=false;
			aimRangePoints=NULL;
			aimAzimuthPoints=NULL;
			aimV=NULL;
			aimStrength=NULL;
			aimLongitude=aimlatitude=NULL;

			sarGMTIAimInfoPosition ps;

			ps.initial();

			sarImageInfo sarImgInfo(ar);

			fmtConvCl fmtConv(ar);

			aimsNum=fmtConv.getBigEndianResult(ps.aimsNum);

			if(aimsNum<=0 || aimsNum>8192)
			{		
				aimsNum=0;
				return;
			}

			//***************************************
			muInfoPosition muPs;
			muPs.initial();
			muInfo muPars(ar);

			// **************************************
			double sarRes = sarImgInfo.sarRes;
			slope_nearest = sarImgInfo.slope_nearest;
			sampoints = sarImgInfo.sampoints; //fmtConv.getResult(sarImagePs.sampoints);		
			prf = sarImgInfo.prf;
			fs = sarImgInfo.fs;
			multipleView = sarImgInfo.multipleView;
			tar_height = muPars.aimH;

			// **************************************
			aimRangePointsFr = new stParInFrame[aimsNum];
			aimAzimuthPointsFr = new stParInFrame[aimsNum];	
			aimVFr = new stParInFrame[aimsNum];	
			aimStrengthFr=new stParInFrame[aimsNum];

			for(int i=0;i<aimsNum;i++)
			{
				//十个字节一个点目标
				//目标azimuth
				aimAzimuthPointsFr[i].startPosion=1028+i*BYTE_PER_TAR_MTI;
				aimAzimuthPointsFr[i].numType=uInt16Enm;

				//目标range
				aimRangePointsFr[i].startPosion=1024+i*BYTE_PER_TAR_MTI;
				aimRangePointsFr[i].numType=uInt32Enm;

				//目标速度	
				aimVFr[i].startPosion=1030+i*BYTE_PER_TAR_MTI;
				aimVFr[i].numType=int16Enm;
				aimVFr[i].unit=1.0/256.0;

				//目标强度
				aimStrengthFr[i].startPosion=1032+i*BYTE_PER_TAR_MTI;
				aimStrengthFr[i].numType=uInt16Enm;
			}

			int ADD_TAR_NUM = 0;
			// added four points space 
			aimRangePoints  =new UINT32[aimsNum+ADD_TAR_NUM];
			aimAzimuthPoints=new UINT16[aimsNum+ADD_TAR_NUM];
			Tar_Azloc=new int[aimsNum+ADD_TAR_NUM];
			Tar_azrev=new char[aimsNum+ADD_TAR_NUM];

			aimV=new float[aimsNum+ADD_TAR_NUM];
			aimStrength=new UINT16[aimsNum+ADD_TAR_NUM];
			aimlatitude=new float[aimsNum+ADD_TAR_NUM];
		    memset(aimlatitude,0,sizeof(float)*(aimsNum+ADD_TAR_NUM));
			aimLongitude=new float[aimsNum+ADD_TAR_NUM];
			memset(aimLongitude,0,sizeof(float)*(aimsNum+ADD_TAR_NUM));

			//标记定位函数
			double dr = C_LIGHT/2.0/fs/1.0e6; // 距离采样间隔
			double az_vel = muPars.plane_ground_v;
		    if(az_vel < 50 || az_vel > 400) az_vel = 200.0;
			double coef_azloc = prf/az_vel/az_vel; 

			// attention !!
			double samps = sampoints;
			double TTp   = sarImgInfo.Tp;
			double dr1 = dr;

			loopNum = cmFuncs.getLoopNum(arImage);
			long rangePoints = cmFuncs.getRangePoints(arImage); // 点迹的辅助数据不包含该信息；
			double r_near = slope_nearest;
			double r_far  = r_near + rangePoints*dr1;

			// STEP 1 读取参数
			for(int i=0;i<aimsNum;i++)
			{
				aimRangePoints[i]	= (UINT32)fmtConv.getBigEndianResult(aimRangePointsFr[i]);
				aimAzimuthPoints[i] = (UINT16)fmtConv.getBigEndianResult(aimAzimuthPointsFr[i]);
				aimStrength[i]		= (UINT16)fmtConv.getBigEndianResult(aimStrengthFr[i]);
				aimV[i]				= -(float)fmtConv.getBigEndianResult(aimVFr[i]);   // added by luo 2016/8/24
				if(abs(aimV[i]) > 100) aimV[i] = 15.0; // Error control
			}

			// STEP2. 点迹凝聚
			int win_ra = GMTI_WIN_RG;   // range should not be larger
			int win_az = GMTI_WIN_AZ;	// after multi-look for big truck 128 ---> 64 
			int checknum = aimsNum;
			
			/*
			FILE *fid = fopen("d:\\azpos.dat","wb");
			fwrite(aimAzimuthPoints,sizeof(UINT16),checknum,fid);
			fclose(fid);

			fid = fopen("d:\\rgpos.dat","wb");
			fwrite(aimRangePoints,sizeof(UINT32),checknum,fid);
			fclose(fid);

			fid = fopen("d:\\tarAmp.dat","wb");
			fwrite(aimStrength,sizeof(UINT16),checknum,fid);
			fclose(fid);
			*/

			if(GMTI_MA_CFAR)
			{
				// 孤立的点可以存在
				checknum = Point_Trace2(aimAzimuthPoints,aimRangePoints,aimStrength,aimV,aimsNum,win_ra,win_az);
			
			}else
			{
				checknum = Point_Trace_Vr(aimAzimuthPoints,aimRangePoints,aimStrength,aimV,aimsNum,win_ra,win_az);
			}

			aimsNum=checknum;
			if(checknum==0) return;

			// ****** STEP3. 目标定位 ********
			double  R0;
			long    az_len = 512*long(64/multipleView);
			for(int i=0;i<checknum;i++)
			{
				// The following code should be checked !!
				R0       = r_near+aimRangePoints[i]*dr1;	//目标斜距  近端斜距+
				Tar_Azloc[i] = int(aimAzimuthPoints[i]-(R0*aimV[i]*coef_azloc/multipleView));		
				Tar_azrev[i] = char(-float(Tar_Azloc[i]<0)+float(Tar_Azloc[i]>(az_len-1)));

				aimV[i] = -aimV[i];  // modified by luo to adapt the new direction 
			   
			}

			// get target GPS 
			img_location_cal(aimLongitude, aimlatitude, az_len, rangePoints, checknum, gps_info, Tar_Azloc, Tar_azrev, aimRangePoints);

			blInitial=true;

		}

		sarGMTIAimInfo::sarGMTIAimInfo(UINT8 *ar)
		{ 
		}

		void sarGMTIAimInfo::img_location_cal(float *logn, float *lati,__int64 na, __int64 nr, int tar_num, double *gps_info, int *tar_az, char *tar_azrev, unsigned int *tar_ra)
		{// OK 0415 check 0518
		   		 
			__int64 i,pos;
			double lon_1  = gps_info[6];
			double lon_2  = gps_info[7];
			double lon_3  = gps_info[8];
			double lon_4  = gps_info[9];

			double lat_1  = gps_info[2];
			double lat_2  = gps_info[3];
			double lat_3  = gps_info[4];
			double lat_4  = gps_info[5];

			long a_1 = 1;
			long a_2 = nr;
			long a_3 = 1;
			long a_4 = nr;

			long b_1 = 1;
			long b_2 = 1;
			long b_3 = na;
			long b_4 = na;

			// 二元矩阵的逆的系数设为A
			double A = double(a_3-a_1)*double(b_2-b_1)-double(b_3-b_1)*double(a_2-a_1);
			if(A==0) return;

			// 在a向上纬度差的系数乘以A后设为La_a,经度差的系数乘以A后设为Lo_a;
			double La_a = (b_2-b_1)*(lat_3-lat_1)-(b_3-b_1)*(lat_2-lat_1);
			double Lo_a = (b_2-b_1)*(lon_3-lon_1)-(b_3-b_1)*(lon_2-lon_1);

			// 在b向上纬度差的系数乘以A后设为La_b,经度差的系数乘以A后设为Lo_b;
			double La_b = (a_3-a_1)*(lat_2-lat_1)-(a_2-a_1)*(lat_3-lat_1);
			double Lo_b = (a_3-a_1)*(lon_2-lon_1)-(a_2-a_1)*(lon_3-lon_1);

			for(i = 0; i < tar_num; i ++)
			{
				{
					logn[i] = lon_1+Lo_a/A*(tar_ra[i]-a_1)+Lo_b/A*(tar_az[i]-b_1);
					lati[i] = lat_1+La_a/A*(tar_ra[i]-a_1)+La_b/A*(tar_az[i]-b_1);
				}  
			}
		}


		sarGMTIAimInfo::~sarGMTIAimInfo()
		{
			if(aimRangePoints!=NULL)
			{
             delete [] aimRangePoints;
			 aimRangePoints=NULL;
			}

			if(aimAzimuthPoints!=NULL)
			{
             delete [] aimAzimuthPoints;
			 aimAzimuthPoints=NULL;
			}

			if(Tar_Azloc!=NULL)
			{
             delete [] Tar_Azloc;
			 Tar_Azloc=NULL;
			}

			if(Tar_azrev!=NULL)
			{
             delete [] Tar_azrev;
			 Tar_azrev=NULL;
			}

			if(aimV!=NULL)
			{
             delete [] aimV;
			 aimV=NULL;
			}

			if(aimlatitude!=NULL)
			{
             delete [] aimlatitude;
			 aimlatitude=NULL;
			}

			if(aimLongitude!=NULL)
			{
             delete [] aimLongitude;
			 aimLongitude=NULL;
			}

			if(aimStrength!=NULL)
			{
             delete [] aimStrength;
			 aimStrength=NULL;
			}
			
		}
		
		int sarGMTIAimInfo::output(FILE * FL)
		{
			 if(blInitial==false)
			 {
				  return -1;
			 }

			 for(UINT32 i=0;i<aimsNum;i++)
			 {
				fprintf(FL,"\n点迹编号：%04d\n",(i+1));
				fprintf(FL,"点迹经度(度)：%.7f\n",aimLongitude[i]);
				fprintf(FL,"点迹纬度(度)：%.7f\n",aimlatitude[i]);
				fprintf(FL,"点迹高度(米)：%.3f\n",tar_height);
				fprintf(FL,"径向速度(米/秒)：%.3f\n",aimV[i]);
			 }

			 return 0;

		}

		int sarGMTIAimInfo::Point_Trace2(UINT16 *Tar_Az,UINT32 *Tar_Ra,UINT16 *Tar_Amp, float *Tar_ph,UINT Tar_Num,int win_ra,int win_az)
		{// 点迹凝聚 OK 10/15 CHECK dev by Yunhua-Luo 

			long   m,n,k,Tar_Num_New;
			UINT16   *Tar_Az_temp =new  UINT16[Tar_Num];
			UINT32   *Tar_Ra_temp =new  UINT32[Tar_Num];
			UINT16  *Tar_Amp_temp=new  UINT16[Tar_Num];  
			float  *Tar_Ph_temp =new  float[Tar_Num];  
			
			for(m=0;m<Tar_Num;m++)
			{
				if(Tar_Amp[m]>0)			
				for(n=0;n<Tar_Num;n++) 
				{
					Tar_Amp[m] = Tar_Amp[m]*float(!((abs(Tar_Az[m]-Tar_Az[n])<=win_az)&&(abs(int(Tar_Ra[m]-Tar_Ra[n]))<=win_ra)&&(Tar_Amp[n]>=Tar_Amp[m])&&(m!=n)&&(Tar_ph[m]*Tar_ph[n]>0)));
					if(Tar_Amp[m] == 0) break;
				}			
			}

			k = 0;
			for(m=0; m<Tar_Num;m++)
			{
				if(Tar_Amp[m]>0) 
				{	
					Tar_Az_temp[k] = Tar_Az[m];
					Tar_Ra_temp[k] = Tar_Ra[m];
					Tar_Amp_temp[k]= Tar_Amp[m];
					Tar_Ph_temp[k] = Tar_ph[m];
					k++;
				}
			}

			Tar_Num_New=k;
			memcpy(Tar_Az,Tar_Az_temp,sizeof(UINT16)*Tar_Num_New);
			memcpy(Tar_Ra,Tar_Ra_temp,sizeof(UINT32)*Tar_Num_New);
			memcpy(Tar_Amp,Tar_Amp_temp,sizeof(UINT16)*Tar_Num_New);
			memcpy(Tar_ph,Tar_Ph_temp,sizeof(float)*Tar_Num_New);
		
			delete []  Tar_Az_temp;
			delete []  Tar_Ra_temp;
			delete []  Tar_Amp_temp; 
			delete []  Tar_Ph_temp;

			return Tar_Num_New;

		}

		int sarGMTIAimInfo::Point_Trace_Vr(UINT16 *Tar_Az,UINT32 *Tar_Ra,UINT16 *Tar_Amp, float *Tar_Vr,UINT Tar_Num,int win_ra,int win_az)
		{// SAR/GMTI点迹凝聚 2016/5/25 check by luo


			 long    m,n,k,Tar_Num_New;
			 long   *Tar_Az_temp=new  long[Tar_Num];
			 long   *Tar_Ra_temp=new  long[Tar_Num];
			 float  *Tar_Amp_temp=new float[Tar_Num];  
			 float  *Tar_Vr_temp =new float[Tar_Num];  
			 k=0;
			 bool flag;
			 double tar_vr;
			 long cnt;
			 int sign_tar_num = 15;
			 long az_loc;

			 for(m=0;m<Tar_Num;m++)
			 {
				tar_vr = 0.0;
				cnt = 0;
				az_loc = 0;
				if(Tar_Amp[m]>0)
				{
					for(n=0;n<Tar_Num;n++) 
					{
						flag = ((abs(int(Tar_Az[m]-Tar_Az[n]))<win_az)&&(abs(int(Tar_Ra[m]-Tar_Ra[n]))<win_ra)&&(m!=n));
						Tar_Amp[n] *= float(!flag);
						if(flag)
						{
							az_loc += Tar_Az[n];
							if(cnt < sign_tar_num)
							{
								tar_vr += Tar_Vr[n];
								cnt++;
							}else
							{
								if(long(Tar_Vr[n])*long(tar_vr)>0)  // same sign
								{
									tar_vr += Tar_Vr[n];
									cnt++;
								}
							}
						}
					}
				}

				if(cnt > 0 && Tar_Amp[m]>0) // can not be just one points 
				{
					float vr_temp = tar_vr/cnt;
					if(abs(vr_temp) > 1.0) // added by Yunhua-Luo 
					{	
					   Tar_Vr_temp[k]  = vr_temp;
					   Tar_Az_temp[k]  = long(az_loc/cnt);
					   Tar_Ra_temp[k]  = Tar_Ra[m];
					   Tar_Amp_temp[k] = Tar_Amp[m];
					   k++;
					}
				}

			  }

			  Tar_Num_New=k;
			  memcpy(Tar_Az,Tar_Az_temp,sizeof(long)*Tar_Num_New);
			  memcpy(Tar_Ra,Tar_Ra_temp,sizeof(long)*Tar_Num_New);
			  memcpy(Tar_Amp,Tar_Amp_temp,sizeof(float)*Tar_Num_New);
			  memcpy(Tar_Vr,Tar_Vr_temp,sizeof(float)*Tar_Num_New);
		
			  delete []  Tar_Az_temp;
			  delete []  Tar_Ra_temp;
			  delete []  Tar_Amp_temp; 
			  delete []  Tar_Vr_temp;

			  return Tar_Num_New;

      }


		GMTIModel::GMTIModel(UINT8* arIn,string str)
		{
			ar=arIn;
			rootPath=str;
			sockM=NULL;
			sockfd=NULL;
			im_ar = NULL;
			multicast_init();
			lastPlaneLon=0;
			lastPlaneLat=0;
			ml2_coef = 1.0;
			gmtiAimPt=NULL;
		}
		
		int GMTIModel::multicast_init()
		{
			//获取组播IP
			std::ifstream fin("iniDzs.txt", std::ios::in);
			if (!fin)
			{
				return -1;
			}

			char ipAr[64] = { 0 };
			char portAr[8] = { 0 };
			fin.getline(ipAr, sizeof(ipAr));
			fin.getline(portAr, sizeof(portAr));
			int lenPortAr = strlen(portAr);
			int portNum = portAr[0] - '0';
			for (int i = 1; i < lenPortAr; i++)
			{
				int iTemp = portAr[i] - '0';
				portNum = 10 * portNum + iTemp;
			}

			if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
			{
				return -1;
			}

			sockfd = WSASocket(AF_INET, SOCK_DGRAM, 0, NULL,
				0, WSA_FLAG_MULTIPOINT_C_LEAF | WSA_FLAG_MULTIPOINT_D_LEAF | WSA_FLAG_OVERLAPPED);

			if (sockfd == INVALID_SOCKET)
			{
				return -1;
			}

			memset(&servaddr, 0, sizeof(servaddr));
			servaddr.sin_family = AF_INET;
			servaddr.sin_addr.s_addr = inet_addr(ipAr);

			servaddr.sin_port = htons(portNum);
			sockM = WSAJoinLeaf(sockfd, (SOCKADDR*)&servaddr, sizeof(servaddr), NULL, NULL, NULL, NULL, JL_BOTH);
			if (sockM == INVALID_SOCKET)
			{
				return -1;
			}

			return 0;
		}

		// SAR/GMTI发送点迹到电子地图
		int  GMTIModel::outPutGMTIData()
		{
			commonFuncs cmFuncs;
			int md=cmFuncs.getSarModel(ar);
			static int aScanMdAngle=0;
			static string dataName=cmFuncs.getDataName(ar,0);
			int loopNum = int(cmFuncs.getLoopNum(ar)); // modified 2016/8/24

			if(md!=3)
			{
				return ERROR_MODEL;
			}

			string gyDatInfoName=rootPath;
			gyDatInfoName.append(dataName+".dat");
			taskInfo task(ar);  
			muInfo mu(ar);
			sarImageInfo imgPars(ar);
			GYParsInfo GYPars(ar);
			
			if(gmtiAimPt==NULL)
			{
				return -1;
			}

			//UINT16 aimNms=gmtiAim.aimsNum;
			UINT16 aimNms=gmtiAimPt->aimsNum;
			
			//用于发送
			char * sendFlName="sarGMTISend.dat";
			FILE * FLSend=fopen(sendFlName,"wb");

			FILE * FL=fopen(gyDatInfoName.c_str(),"ab");

			//数据包信息
			UINT32 *packetInfo=new UINT32[3];
			
			packetInfo[0]=0X55AA55AA;
			fwrite(&(packetInfo[0]),sizeof(UINT32),1,FL);
			fwrite(&(packetInfo[0]),sizeof(UINT32),1,FLSend);
			
			packetInfo[1]=3; // SAR-MTI点迹
			fwrite(&(packetInfo[1]),sizeof(UINT32),1,FL);
			fwrite(&(packetInfo[1]),sizeof(UINT32),1,FLSend);
			
			packetInfo[2]=216+aimNms*40; 
            fwrite(&(packetInfo[2]),sizeof(UINT32),1,FL);
			fwrite(&(packetInfo[2]),sizeof(UINT32),1,FLSend);

			UINT16 taskCode=(UINT16)(task.missionCodes);
            //12
			fwrite(&taskCode,sizeof(UINT16),1,FL);//任务代号
			fwrite(&taskCode,sizeof(UINT16),1,FLSend);//任务代号
			//
			UINT8 planeType=7;

			fwrite(&planeType,sizeof(UINT8),1,FL);//飞行器类型
			fwrite(&planeType,sizeof(UINT8),1,FLSend);//飞行器类型

			UINT8 planePP=0;

			fwrite(&planePP,sizeof(UINT8),1,FL);//飞机批号
			fwrite(&planePP,sizeof(UINT8),1,FLSend);//飞机批号

			UINT16 planeCode=(UINT16)(task.planeNum);
			
			fwrite(&planeCode,sizeof(UINT16),1,FL);//飞机号 
			fwrite(&planeCode,sizeof(UINT16),1,FLSend);//飞机号
             //18
			UINT8 picCompress=0;
			fwrite(&picCompress,sizeof(UINT8),1,FL);//图像压缩比
			fwrite(&picCompress,sizeof(UINT8),1,FLSend);//图像压缩比

			UINT8 transType=(UINT8)task.transType;
			fwrite(&transType,sizeof(UINT8),1,FL);//传输方式
			fwrite(&transType,sizeof(UINT8),1,FLSend);//传输方式

			UINT8 loadType=3;
			fwrite(&loadType,sizeof(UINT8),1,FL);
			fwrite(&loadType,sizeof(UINT8),1,FLSend);

			UINT32 loadCode=0;
			fwrite(&loadCode,sizeof(UINT32),1,FL);
			fwrite(&loadCode,sizeof(UINT32),1,FLSend);
            //25
			UINT16 powerOn=(UINT16)GYPars.powerOnTimes;
			fwrite(&powerOn,sizeof(UINT16),1,FL);
			fwrite(&powerOn,sizeof(UINT16),1,FLSend);

			UINT32 muCode=0;
			fwrite(&muCode,sizeof(UINT32),1,FL);
			fwrite(&muCode,sizeof(UINT32),1,FLSend);
            //31
			UINT16 year=(UINT16)mu.date_year;
			fwrite(&year,sizeof(UINT16),1,FL);
			fwrite(&year,sizeof(UINT16),1,FLSend);
             //33
			UINT8 month=(UINT8)mu.date_month;
			fwrite(&month,sizeof(UINT8),1,FL);
			fwrite(&month,sizeof(UINT8),1,FLSend);

			UINT8 day=(UINT8)mu.date_day;
			fwrite(&day,sizeof(UINT8),1,FL);
			fwrite(&day,sizeof(UINT8),1,FLSend);

			UINT8 hour=(UINT8)mu.time_hour;
			fwrite(&hour,sizeof(UINT8),1,FL);
			fwrite(&hour,sizeof(UINT8),1,FLSend);

			UINT8 minutes=(UINT8)mu.time_minutes;
			fwrite(&minutes,sizeof(UINT8),1,FL);
			fwrite(&minutes,sizeof(UINT8),1,FLSend);

			UINT8 seconds=(UINT8)mu.time_second;
			fwrite(&seconds,sizeof(UINT8),1,FL);
			fwrite(&seconds,sizeof(UINT8),1,FLSend);

			UINT16 mseconds=(UINT16)mu.time_m_second;
			fwrite(&mseconds,sizeof(UINT16),1,FL);
			fwrite(&mseconds,sizeof(UINT16),1,FLSend);
             //40
			double plane_Long=(double)mu.plane_longitude;
			fwrite(&plane_Long,sizeof(double),1,FL);
			fwrite(&plane_Long,sizeof(double),1,FLSend);
			double plane_Lat=(double)mu.plane_latitude;
			fwrite(&plane_Lat,sizeof(double),1,FL);//
			fwrite(&plane_Lat,sizeof(double),1,FLSend);//

			float * flt=new float[22];
             //56
			flt[0]=(float)mu.plane_height;
			fwrite(&(flt[0]),sizeof(float),1,FL);
			fwrite(&(flt[0]),sizeof(float),1,FLSend);

			flt[1]=(float)mu.plane_aim_height;
			fwrite(&(flt[1]),sizeof(float),1,FL);
			fwrite(&(flt[1]),sizeof(float),1,FLSend);

			flt[2]=(float)mu.plane_direction_angle;
			fwrite(&(flt[2]),sizeof(float),1,FL);
			fwrite(&(flt[2]),sizeof(float),1,FLSend);

			flt[3]=(float)mu.plane_departure_angle;
			fwrite(&(flt[3]),sizeof(float),1,FL);
			fwrite(&(flt[3]),sizeof(float),1,FLSend);

			flt[4]=(float)mu.plane_departure_angle;
			fwrite(&(flt[4]),sizeof(float),1,FL);
			fwrite(&(flt[4]),sizeof(float),1,FLSend);

			//俯仰角
			flt[5]=(float)mu.plane_dive_angle;
			fwrite(&(flt[5]),sizeof(float),1,FL);
			fwrite(&(flt[5]),sizeof(float),1,FLSend);

			flt[6]=(float)mu.plane_dive_angle_V;
			fwrite(&(flt[6]),sizeof(float),1,FL);
			fwrite(&(flt[6]),sizeof(float),1,FLSend);

			flt[7]=(float)mu.plane_dive_angle_a;
			fwrite(&(flt[7]),sizeof(float),1,FL);
			fwrite(&(flt[7]),sizeof(float),1,FLSend);

			//横滚角
			flt[8]=(float)mu.plane_hor_angle;
			fwrite(&(flt[8]),sizeof(float),1,FL);
			fwrite(&(flt[8]),sizeof(float),1,FLSend);

            flt[9]=(float)mu.plane_hor_angle_v;
			fwrite(&(flt[9]),sizeof(float),1,FL);
			fwrite(&(flt[9]),sizeof(float),1,FLSend);

			flt[10]=(float)mu.plane_hor_angle_a;
			fwrite(&(flt[10]),sizeof(float),1,FL);
			fwrite(&(flt[10]),sizeof(float),1,FLSend);

			flt[11]=(float)mu.plane_departure_angle;
			fwrite(&(flt[11]),sizeof(float),1,FL);
			fwrite(&(flt[11]),sizeof(float),1,FLSend);

			flt[12]=(float)mu.plane_de_flow_angle;
			fwrite(&(flt[12]),sizeof(float),1,FL);
			fwrite(&(flt[12]),sizeof(float),1,FLSend);

			flt[13]=(float)mu.plane_ground_v;
			fwrite(&(flt[13]),sizeof(float),1,FL);
			fwrite(&(flt[13]),sizeof(float),1,FLSend);

            flt[14]=(float)mu.plane_noair_v;
			fwrite(&(flt[14]),sizeof(float),1,FL);
			fwrite(&(flt[14]),sizeof(float),1,FLSend);

			flt[15]=(float)mu.plane_point_v;
			fwrite(&(flt[15]),sizeof(float),1,FL);
			fwrite(&(flt[15]),sizeof(float),1,FLSend);
			//
			flt[16]=(float)mu.plane_east_v;
			fwrite(&(flt[16]),sizeof(float),1,FL);
			fwrite(&(flt[16]),sizeof(float),1,FLSend);

			flt[17]=(float)mu.plane_north_v;
			fwrite(&(flt[17]),sizeof(float),1,FL);
			fwrite(&(flt[17]),sizeof(float),1,FLSend);

			flt[18]=(float)mu.plane_up_v;
			fwrite(&(flt[18]),sizeof(float),1,FL);
			fwrite(&(flt[18]),sizeof(float),1,FLSend);

			flt[19]=(float)mu.plane_east_a;
			fwrite(&(flt[19]),sizeof(float),1,FL);
			fwrite(&(flt[19]),sizeof(float),1,FLSend);

			flt[20]=(float)mu.plane_north_a;
			fwrite(&(flt[20]),sizeof(float),1,FL);
			fwrite(&(flt[20]),sizeof(float),1,FLSend);

			flt[21]=(float)mu.plane_up_a;
			fwrite(&(flt[21]),sizeof(float),1,FL);
			fwrite(&(flt[21]),sizeof(float),1,FLSend);

			if(flt!=NULL)
			{
				delete []flt;
				flt=NULL;
			}
             //22*4+56=144
			//天线帧编号

			UINT32 frameNum= loopNum;
			fwrite(&frameNum,sizeof(UINT32),1,FL);
			fwrite(&frameNum,sizeof(UINT32),1,FLSend);

			//波位数
			UINT16 waveNum=0;
			fwrite(&waveNum,sizeof(UINT16),1,FL);
			fwrite(&waveNum,sizeof(UINT16),1,FLSend);

			//波位号
			UINT16 waveCode=(UINT16)GYPars.waveCodes;
			fwrite(&waveCode,sizeof(UINT16),1,FL);
			fwrite(&waveCode,sizeof(UINT16),1,FLSend);

			//工作频段 侧视方式  工作模式 工作子模式
			UINT8 * FreqLS=new UINT8[4];
			FreqLS[0]=0;
			fwrite(&(FreqLS[0]),sizeof(UINT8),1,FL);
			fwrite(&(FreqLS[0]),sizeof(UINT8),1,FLSend);

			if(mu.rAngle<0) // 右侧视
			{
				FreqLS[1] = 1;
			}else           // 左侧视
			{
				FreqLS[1] = 0;
			}

			//FreqLS[1]=0;
			fwrite(&(FreqLS[1]),sizeof(UINT8),1,FL);
			fwrite(&(FreqLS[1]),sizeof(UINT8),1,FLSend);

			FreqLS[2]=3; // SAR/GMTI
			fwrite(&(FreqLS[2]),sizeof(UINT8),1,FL);
			fwrite(&(FreqLS[2]),sizeof(UINT8),1,FLSend);

			FreqLS[3]=0;
			fwrite(&(FreqLS[3]),sizeof(UINT8),1,FL);
			fwrite(&(FreqLS[3]),sizeof(UINT8),1,FLSend);

			//156
			UINT32 *fNDistance=new UINT32[2];

			fNDistance[1]=(UINT32 )GYPars.RFar;
			fNDistance[0]=(UINT32 )GYPars.RNear;//dbRNear

			fwrite(&(fNDistance[1]),sizeof(UINT32),1,FL);
			fwrite(&(fNDistance[0]),sizeof(UINT32),1,FL);

			fwrite(&(fNDistance[1]),sizeof(UINT32),1,FLSend);
			fwrite(&(fNDistance[0]),sizeof(UINT32),1,FLSend);
		
			if(fNDistance!=NULL)
			{
				delete []fNDistance;
				fNDistance=NULL;
			}

			float * scanPar=new float[8];

			//天线帧扫描中心角		
			scanPar[0]=(float)((int)(mu.plane_direction_angle-mu.plane_departure_angle-imgPars.look_Side*90+360)%360);
			//cmFuncs.logRecords("SAR/GMTI天线帧扫描中心角：",scanPar[0]);
			fwrite(&(scanPar[0]),sizeof(float),1,FL);
			fwrite(&(scanPar[0]),sizeof(float),1,FLSend);
			
			//天线帧扫描范围
			scanPar[1]=2*(float)GYPars.scanScope;   // luo
			scanPar[1] = imgPars.beam_horz_width; // luo
			
			fwrite(&(scanPar[1]),sizeof(float),1,FL);
			fwrite(&(scanPar[1]),sizeof(float),1,FLSend);

			//方位波束中心角
			scanPar[2] = scanPar[0]; //(float)((int)(mu.plane_direction_angle-imgPars.look_Side*90+360)%360); // by luoyunhua 2016/06/06

			fwrite(&(scanPar[2]),sizeof(float),1,FL);
			fwrite(&(scanPar[2]),sizeof(float),1,FLSend);

            //方位波束宽度 ***************imgPars.beam_horz_width
			scanPar[3]=(float)imgPars.beam_horz_width;

			fwrite(&(scanPar[3]),sizeof(float),1,FL);
			fwrite(&(scanPar[3]),sizeof(float),1,FLSend);

			//方位向扫描步进
			scanPar[4]=(int)imgPars.beam_horz_width;
			fwrite(&(scanPar[4]),sizeof(float),1,FL);
			fwrite(&(scanPar[4]),sizeof(float),1,FLSend);

			//俯仰波束中心角
			scanPar[5]=imgPars.R_angle_new;
			fwrite(&(scanPar[5]),sizeof(float),1,FL);
			fwrite(&(scanPar[5]),sizeof(float),1,FLSend);

			//俯仰波束宽度
			scanPar[6]=imgPars.beam_R_width;
			fwrite(&(scanPar[6]),sizeof(float),1,FL);
			fwrite(&(scanPar[6]),sizeof(float),1,FLSend);

			scanPar[7]=0;
			fwrite(&(scanPar[7]),sizeof(float),1,FL);
			fwrite(&(scanPar[7]),sizeof(float),1,FLSend);

			if(scanPar!=NULL)
			{
				delete []scanPar;
				scanPar=NULL;
			}		

			UINT16 resPulseNum=(UINT16)GYPars.pulseResident;
			fwrite(&resPulseNum,sizeof(UINT16),1,FL);
			fwrite(&resPulseNum,sizeof(UINT16),1,FLSend);

			UINT16 resTime=0;
			fwrite(&resTime,sizeof(UINT16),1,FL);
			fwrite(&resTime,sizeof(UINT16),1,FLSend);
		    //分辨率
			float resolv=0;
			fwrite(&resolv,sizeof(float),1,FL);
			fwrite(&resolv,sizeof(float),1,FLSend);

			UINT32 PRF=(UINT32)GYPars.prf;
			fwrite(&PRF,sizeof(UINT32),1,FL);
			fwrite(&PRF,sizeof(UINT32),1,FLSend);

			UINT16 nullPs=0;
			fwrite(&nullPs,sizeof(UINT16),1,FL);
			fwrite(&nullPs,sizeof(UINT16),1,FLSend);

			fwrite(&aimNms,sizeof(UINT16),1,FL);
			fwrite(&aimNms,sizeof(UINT16),1,FLSend);

			for(int p=0;p<(gmtiAimPt->aimsNum);p++)
			{

			   UINT32 pointCodes=(UINT32)p;
			   fwrite(&pointCodes,sizeof(UINT32),1,FL);
			   fwrite(&pointCodes,sizeof(UINT32),1,FLSend);
               //序号
			   double pointLong=gmtiAimPt->aimLongitude[p];  //经度
			   fwrite(&pointLong,sizeof(double),1,FL);
			   fwrite(&pointLong,sizeof(double),1,FLSend);

			   double pointLat=gmtiAimPt->aimlatitude[p];  //纬度
			   fwrite(&pointLat,sizeof(double),1,FL);
			   fwrite(&pointLat,sizeof(double),1,FLSend);

			   float pointHeight=gmtiAimPt->tar_height;
			   fwrite(&pointHeight,sizeof(float),1,FL);
			   fwrite(&pointHeight,sizeof(float),1,FLSend);

			   float point_P_v=gmtiAimPt->aimV[p];
			   fwrite(&point_P_v,sizeof(float),1,FL);
			   fwrite(&point_P_v,sizeof(float),1,FLSend);

			   UINT32 img_LoopNum=gmtiAimPt->loopNum; 
			   fwrite(&img_LoopNum,sizeof(UINT32),1,FL);
			   fwrite(&img_LoopNum,sizeof(UINT32),1,FLSend);

			   UINT32 rg_Num=gmtiAimPt->aimRangePoints[p];
			   if(mu.rAngle>0) rg_Num = img_col - rg_Num;
			   fwrite(&rg_Num,sizeof(UINT32),1,FL);
			   fwrite(&rg_Num,sizeof(UINT32),1,FLSend);

			   UINT32 az_num=img_row - long(gmtiAimPt->Tar_Azloc[p]*ml2_coef); // after locating
				
			   if(ml2_coef >= 1) 
			   az_num=img_row - long(gmtiAimPt->Tar_Azloc[p]); // after locating 
			  
			   fwrite(&az_num,sizeof(UINT32),1,FL);
			   fwrite(&az_num,sizeof(UINT32),1,FLSend);
			}

			UINT32 packetTail=0X0D0D0D0D;
			fwrite(&packetTail,sizeof(UINT32),1,FL);
			fwrite(&packetTail,sizeof(UINT32),1,FLSend);

			fclose(FL);
			fclose(FLSend);

			FLSend=fopen(sendFlName,"rb");

			if(FLSend==NULL)
			{
				return -1;
			}

			fseek(FLSend,0, SEEK_END);
			int file_size = ftell(FLSend);
			fseek(FLSend,0,SEEK_SET);

			char * buffer=new char[file_size];			
			fread(buffer,sizeof(char),file_size,FLSend);
			fclose(FLSend);
			
		    int resTemp=sendto(sockfd, buffer,file_size, 0,  
				(struct sockaddr *)&servaddr, sizeof(sockaddr));

			return resTemp;

		}

		// add by zhang : add width and height information 
		int GMTIModel::outPutGMTIInfo(int loopNum, int nImgWidth, int nImgHeight, float az_ml2_coef)
		{// CHECK OK 2016/06/07

			//融合，对收到的图像进行计算
			static int cntRecvpLoop=0;
			static char * imgAr=NULL;
			static int  tifWidthLast=0;
			int tifWidth;

			static int stTemp=0;
			static char mergePath[4][128];

			commonFuncs cmFuncs;
			int md=cmFuncs.getSarModel(ar);
			
			if(md!=3)
			{
				//cmFuncs.logRecords("广域GMTI中的sarModel错误",0);
				return ERROR_MODEL;
			}

			//判断是点还是图
			string namePicStr=rootPath;
			
			//拆分后的路径
			string nameStr;
			if(overlay) //
			{
				nameStr = rootPath + cmFuncs.getMarkPicName(im_ar,loopNum)+".txt";

			}else
			{
				nameStr = rootPath + cmFuncs.getRawPicName(im_ar,loopNum) + ".txt";
			}
			int pathLen=nameStr.length();
	
			char *nameCh=new char[pathLen+1];

			for(int i=0;i<pathLen;i++)
			{
				nameCh[i]=nameStr[i];
			}
			nameCh[pathLen]='\0';

			FILE * FL=fopen(nameCh,"a");

			if(FL==NULL)
			{
				delete [] nameCh;
				nameCh=NULL;
				return ERROR_PATH;
			}			

			taskInfo task(ar_last);  // from ar_last to get imcompress ratio value 
			task.output(FL);

			muInfo mu(ar);
			sarImageInfo si(ar);			// ar for aim file
			sarImageInfo si_end(ar_last);   // ar_last for image file

			si.img_length	  *= (nImgHeight/512);
			si_end.img_length *= (nImgHeight/512);

			int picNum = (nImgHeight/512);
			if(picNum > 1)
			{
				si_end.plane_latitude  += (si_end.plane_latitude - si.plane_latitude)*1.0/(picNum-1);
				si_end.plane_longitude += (si_end.plane_longitude - si.plane_longitude)*1.0/(picNum-1);
			}

			// Calcuate the next GPS point
			double lati_s,logn_s,lati_e,logn_e;
			lati_s = si.plane_latitude;
			logn_s = si.plane_longitude; 
			lati_e = si_end.plane_latitude;
			logn_e = si_end.plane_longitude; 

			// **************************************
			si_end.sarImageLocating(lati_s,logn_s,lati_e,logn_e);

			// 点顺序： 左上、右上、左下、右下
			// ******* for geo-tiff generation ********
			
			img_latis[0] = si_end.left_top_latitude;
			img_latis[1] = si_end.right_up_latitude;
			img_latis[2] = si_end.left_down_latitude;
			img_latis[3] = si_end.right_down_latitude;

			img_logns[0] = si_end.left_top_longitude;
			img_logns[1] = si_end.right_up_longitude;
			img_logns[2] = si_end.left_down_longitude;
			img_logns[3] = si_end.right_down_longitude;
			
			/* ==== add by zhang : modify width and height information ==== */
			si_end.img_cols = nImgWidth;
			si_end.img_rows = nImgHeight;
			
			memcpy(gps_info,si_end.gps_info,sizeof(double)*10);
			// *******************************************

			si_end.loop_num = loopNum;
			si_end.ml2_coef = az_ml2_coef;  // pass the value from GMTIMd class 

			si_end.output(FL);		
			mu.output(FL,0);
			fclose(FL);

			delete [] nameCh;

			return 0;

		}
		

		// ******** WAS-GMTI track output class *********

		// WAS-GMTI/参数获取

		GYParsInfo::GYParsInfo(UINT8 *ar)
		{// check by luo 2021/01/18

			static double scanAnglePre=0;
			if(ar==NULL)
			{
				return;
			}

			GYParsInfoPosition ps;
			ps.initial();
			muInfo mu(ar);
			sarImageInfo imgPars(ar); // OK
			fmtConvCl fmtConv(ar);

			fly_ang = mu.plane_direction_angle;
			powerOnTimes=fmtConv.getResult(ps.powerOnTimes);
			frameCodes=fmtConv.getResult(ps.frameCodes);
			waveCodes=fmtConv.getResult(ps.waveCodes);
			scanCenterAngle=fmtConv.getResult(ps.scanCenterAngle);
			scanScope=fmtConv.getResult(ps.scanScope);			
			pulseResident=fmtConv.getResult(ps.pulseResident);	
			scanAngle=fmtConv.getResult(ps.scanAngle);

			Cfar_th_dB = mu.cfar_th_dB;
			
			// new added by Luo 2021/01/19
			float theta = -scanScope;
			int k = 0;
			for(k=0; k<200; k++)
			{
				float scan_step = 0.03124/0.768*0.731*180/PI/cos(theta*PI/180.0);
				theta += scan_step;
				if(theta > scanScope) break;
			}
			waveNum = k;

			scanAnglePre=scanAngle;

			loadCode=0x67;
			loadType=3;

			band = imgPars.band;
			fs  = imgPars.fs;
			prf = imgPars.prf; 
			RNear = imgPars.slope_nearest;
			RFar  = imgPars.slope_far;
			rg_beam_width = imgPars.beam_R_width;
			rg_ang = imgPars.rsAngle;
			
			if(band==80)
				resvAb=10;
			else
                resvAb=20;

			//方位波束中心角--正北指向
			double azTemp=fmtConv.getResult(ps.azimuthCenterAngle);
			azimuthCenterAngle=(float)((int)(mu.plane_direction_angle-mu.plane_departure_angle-imgPars.look_Side*90+imgPars.look_Side*azTemp+360)%360);
			
			// updated by Yunhua-Luo 12/17;
			double constTheta=0.731*0.03125*180/(0.768*PI);
			azimuthAngle=constTheta/cos(scanAngle*PI/180);
		
			scanCycle = double(pulseResident)*waveNum/prf;

		}
		
		GYParsInfo::~GYParsInfo()
		{

		}
		
		int GYParsInfo::output(FILE *FL)
		{// OK 2016/06/03

			fprintf(FL,"天线帧编号:%06d\n",int(frameCodes));
			fprintf(FL,"波位数:%04d\n",int(waveNum));
			fprintf(FL,"波位号:%04d\n",int(waveCodes));

			fprintf(FL,"工作频段(0=X,1=L,2=P,3=P+L,4=Ku):00\n");
			if(rg_ang>0)
			{
				fprintf(FL,"侧视方式(0=左侧视,1=右侧视):00\n");  // modify by zhang
			}
			else
			{
				fprintf(FL,"侧视方式(0=左侧视,1=右侧视):01\n");  // modify by zhang
			}

			fprintf(FL,"工作模式(00=GMTI,01=MMTI,02=AMTI,03=SAR/GMTI同时模式): 00\n");
			fprintf(FL,"工作子模式(0=广域,1=扇区,2=跟踪,9=空缺: 00\n");
			fprintf(FL,"最大作用距离(米): %.0f\n",RFar);
			fprintf(FL,"最小作用距离(米): %.0f\n",RNear);
			fprintf(FL,"天线帧扫描中心角(度):%.4f\n",scanCenterAngle);
			fprintf(FL,"天线帧扫描范围(度):%.4f\n",2*scanScope);
			fprintf(FL,"方位波束中心角(度):%.4f\n",azimuthCenterAngle);
			fprintf(FL,"方位波束宽度(度): %.4f\n",azimuthAngle);
			fprintf(FL,"方位向扫描步进(度): %.4f\n",scanStep);
			fprintf(FL,"俯仰波束中心角(度):%.4f\n",rg_ang);
			fprintf(FL,"俯仰波束宽度(度):%.4f\n",rg_beam_width);	
			fprintf(FL,"俯仰向扫描步进(度):0.0000\n");	
			fprintf(FL,"驻留脉冲数:%04d\n",int(pulseResident));
			fprintf(FL,"波束驻留时间(毫秒):%05d\n",long(1000.0*pulseResident/prf));		
			fprintf(FL,"分辨率(米):%.1f\n",resvAb);
			fprintf(FL,"重复频率(Hz): %05d\n",int(prf));
			fprintf(FL, "CFAR检测门限值(dB):%.3f\n", Cfar_th_dB);

			return 0;
		}

        // WAS-GMTI目标定位求解 2021/07/15
        NewGYAimInfo::NewGYAimInfo(UINT8* ar)
		{
			commonFuncs cmFuncs;

			lstHeader.gyAimInfoPt=NULL;
			lstHeader.numAll=0;

			int PowerOnTime = cmFuncs.getPowerOnTime(ar);
			NewGYAimInfoPosition ps;
			ps.initial();

			fmtConvCl fmtConv(ar);	
			GYParsInfo GYPs(ar);
			muInfo  mu(ar);
			sarImageInfo imgInfo(ar);

			if (imgInfo.slope_nearest < mu.plane_aim_height)
				return;

			//imgInfo.time_poweron;
			memcpy(head,ar,512);
			scanScope = GYPs.scanScope;
			aimsNum = fmtConv.getBigEndianResult(ps.aimsNum);  // 实际的目标数目
			aimsAll = fmtConv.getBigEndianResult(ps.aimsAll);			// 所有目标数   
			pulseResident = fmtConv.getResult(ps.pulseResident);

			
			float prf = imgInfo.prf;
			float sarRes = imgInfo.sarRes;  // added by Luo 2020/03/04
			if(sarRes == 20)   
				is_sea = true;
			else 
				is_sea = false;
			double dbFrameCode = GYPs.frameCodes;

			float true_fly = atan2(mu.plane_east_v,mu.plane_north_v)*180/PI;
			if(true_fly < 0) true_fly +=360;
			float yaw = mu.plane_direction_angle-true_fly;

			float yaw_angle = mu.plane_departure_angle; // 横滚角
			float roll_angle = mu.plane_hor_angle;

			if(abs(roll_angle) > 30 || abs(yaw) > 30)  // modified 
			{
				aimsNum = 0;
				cmFuncs.logRecords("Fly yaw angle Error : ",double(yaw));
				return;
			}

			if(aimsNum <= 0 || aimsNum > 1024 ) // modified by luo
			{
				aimsNum = 0;
				cmFuncs.logRecords("Number of Targets Error : ",double(aimsNum));
				return;
			}

			if(pulseResident > 8192 || pulseResident < 8)
			{
				pulseResident=0;
				cmFuncs.logRecords("PulseResident Error : ",double(pulseResident));
				return;
			}

			// **** Extract target information according to the 2019/09/06 for FPGA V4 ****
			int loc = 1536; //4096+512+512 
			int range_points = ar[512 + 242 * 2+1] * 256 + ar[512 + 242* 2 ];
			pulseResident    =  ar[512 + 244 * 2+1] * 256 + ar[512 + 244 * 2 ];
			float fdc_est      =  (ar[512 + 246 * 2+1] * 256 + ar[512 + 246 * 2] -pulseResident/2)*prf/pulseResident;
			aimsNum          =  ar[512] * 256 + ar[512 + 1];

			double *pdouble;
			float nosie_level;
			pdouble = (double *)(&ar[512 + 247 * 2]);
			float ch1_ph = pdouble[0];
			float ch2_ph = pdouble[1];

			memset(aimPars,0,sizeof(double)*GYAIMLENMAX);
			for(int i=0;i<aimsNum;i++)		// checked by luo 2020/11/25 
			{
				aimPars[i*INFO_NUM]     = (ar[loc+i*16+0]*256.0+ar[loc+i*16+1])*65536.0+ar[loc+i*16+2]*256+ar[loc+i*16+3];  // 目标幅度 
 				aimPars[i*INFO_NUM+1] = (ar[loc+i*16+4]*256.0+ar[loc+i*16+5])*65536.0+ar[loc+i*16+6]*256+ar[loc+i*16+7];  // 噪声平均值 

				aimPars[i*INFO_NUM+2] = ar[loc+i*16+10]*256+ar[loc+i*16+11];  // 距离向位置
				aimPars[i*INFO_NUM+3] = ar[loc+i*16+12]*256+ar[loc+i*16+13];  // 方位向位置

				aimPars[i*INFO_NUM+4] = Bit16ToDot(ar[loc+i*16+14]*256+ar[loc+i*16+15]);		//目标干涉相位															  // 目标的干涉相位
				aimPars[i*INFO_NUM+5] = 20 * log10(abs(aimPars[i*INFO_NUM]   / aimPars[i*INFO_NUM+1] )); // 目标信噪比

				aimPars[i*INFO_NUM+6] = ar[loc+i*16+8]*256+ar[loc+i*16+9]; // 目标宽度

			}

			/*
			FILE *fw1;
			char rtfilename1[128];
			sprintf(rtfilename1, "d:\\MtiDotInfoV4_%04d%02d%02d_Pwr%0_Burst%d_Frame%d.dat", int(mu.date_year), int(mu.date_month), int(mu.date_day), int(GYPs.waveCodes),int(GYPs.frameCodes));
			fw1 = fopen(rtfilename1, "wb");
			fwrite(aimPars, sizeof(double), aimsNum*INFO_NUM, fw1);
			fclose(fw1);
			*/

			FILE *fww;
			char rtfilename[128];
			sprintf(rtfilename, "dzsmti_%04d%02d%02d_pwr%02d_rtrecv.dat",int(mu.date_year),int(mu.date_month),int(mu.date_day), int(PowerOnTime));
			fww = fopen(rtfilename, "ab");
			fwrite(ar, 1, BURST_INFO_LEN, fww);
			fclose(fww);

			// *************** Target locating *************
			sys_par pars;
			pars.wave_no = GYPs.waveCodes;	
			pars.frame_no = GYPs.frameCodes;
			
			pars.B_Na = pulseResident;				  //驻留脉冲数
			pars.R_near = imgInfo.slope_nearest;  //最近斜距离 added by luo 20190615
			pars.tar_height = imgInfo.aim_height;
			pars.prf = prf;
			pars.ch1_ph = ch1_ph;
			pars.ch2_ph = ch2_ph;
			pars.fdc_est = fdc_est;
			pars.Nr = range_points;
	
			pars.az_vel = sqrt(mu.plane_north_v*mu.plane_north_v+mu.plane_east_v*mu.plane_east_v);
			if(pars.az_vel < 1 || pars.az_vel > 1000) 
				pars.az_vel = 200.0;

			pars.scan_ang = GYPs.scanAngle;		// 角度
			pars.elev_ang = mu.rAngle_pt;		   //imgInfo.rAngle_pt; //imgInfo.rsAngle;           // 角度
			pars.yaw_ang = yaw;   // 角度

			pars.lati = mu.plane_latitude;   //载机纬度
			pars.logn = mu.plane_longitude;  //载机经度

			pars.e_v = mu.plane_east_v;
			pars.n_v = mu.plane_north_v;
			pars.u_v = mu.plane_up_v;
			pars.fly_ang = mu.plane_direction_angle; // 载机航向角-偏航角=真航向（飞机实际飞行方向）
			pars.Fs = (imgInfo.fs*1.0e6);  // 31Mhz
			pars.href = mu.plane_height;
			pars.ts = mu.date_day*24*3600.0+mu.time_hour*3600.0+mu.time_minutes*60.0+mu.time_second+mu.time_m_second/1000.0;
			
			pars.lookside = imgInfo.look_Side;  // 左--1，右 -- -1;
			pars.fc = double(9.6e9);
			pars.Br = (GYPs.band*1.0e6);
			pars.D = 0.384;
			pars.is_sea = is_sea;
			
			cmFuncs.logRecords("Target Number Before : ",double(aimsNum));

			aimsNum = WASmti_locating_v2020(aimPars,aimsNum,pars);

			cmFuncs.logRecords("Target Number After : ",double(aimsNum));

			return;

		}
		
		NewGYAimInfo::~NewGYAimInfo()
		{
            gyAimInfoList * lstDel=lstHeader.gyAimInfoPt;
			while(lstDel!=NULL)
			{
				 gyAimInfoList * Temp=lstDel;
				 lstDel=lstDel->next;

				 delete Temp;
			}

			lstHeader.gyAimInfoPt=NULL;
		}

		template <class T>
		inline  double NewGYAimInfo::min_val(T *val,__int64 num,__int64 *loc)
		{
			long    i;
			long    min_loc=0;
			double  min_val0;
			if(num<0) return 0.0;
			min_val0=val[0];
			for(i=0;i<num;i++)
			{
				if(val[i]<min_val0) 
				{	  
				min_val0=val[i];
				min_loc=i;
				}
				}
			*loc=min_loc;
			return   min_val0;
		}

		template <class T>
		inline void NewGYAimInfo::CircShift(T *data,long data_len,int mov_shift)
		{// CHECK BY luo
     
		T *buf=new T[data_len];
		if(mov_shift<0) mov_shift+=data_len;
		memcpy(buf+mov_shift,data,sizeof(T)*(data_len-mov_shift)); 
		memcpy(buf,data+data_len-mov_shift,sizeof(T)*(mov_shift));
		memcpy(data,buf,sizeof(T)*data_len);
		delete buf; 
		}

	    template <class T>
	    void NewGYAimInfo::Write2Double(char* file_out,T* pdata,__int64 N)
		{//OK

			FILE * hFile;
			if((hFile = fopen(file_out,"wb")) == NULL)
			{
			printf("Can't open file to read!!!!!\n");
			return;
			}
			__int64 len = fwrite(pdata,sizeof(T),N,hFile);
			fclose(hFile);

		}

        int NewGYAimInfo::Point_Trace(long *Tar_Az,long *Tar_Ra,float *Tar_Amp, float *Tar_ph,long Tar_Num,int win_ra,int win_az)
		{// 点迹凝聚 OK 10/15 CHECK dev by Yunhua-Luo 

			long   m,n,k,Tar_Num_New;
			long   *Tar_Az_temp =new  long[Tar_Num];
			long   *Tar_Ra_temp =new  long[Tar_Num];
			float  *Tar_Amp_temp=new  float[Tar_Num];  
			float  *Tar_Ph_temp =new  float[Tar_Num];  
			k=0;

			for(m=0;m<Tar_Num;m++)
			{
				if(Tar_Amp[m]>0)			
				for(n=0;n<Tar_Num;n++) 
				{
					Tar_Amp[m]*=float(!((abs(Tar_Az[m]-Tar_Az[n])<win_az)&&(abs(Tar_Ra[m]-Tar_Ra[n])<win_ra)&&(Tar_Amp[n]>Tar_Amp[m])&&(m!=n)));
					if(Tar_Amp[m]==0) break;
				}

				if(Tar_Amp[m]>0) 
				{	
					Tar_Az_temp[k] = Tar_Az[m];
					Tar_Ra_temp[k] = Tar_Ra[m];
					Tar_Amp_temp[k]= Tar_Amp[m];
					Tar_Ph_temp[k] = Tar_ph[m];
					k++;
				}
			}

			Tar_Num_New=k;
			memcpy(Tar_Az,Tar_Az_temp,sizeof(long)*Tar_Num_New);
			memcpy(Tar_Ra,Tar_Ra_temp,sizeof(long)*Tar_Num_New);
			memcpy(Tar_Amp,Tar_Amp_temp,sizeof(float)*Tar_Num_New);
			memcpy(Tar_ph,Tar_Ph_temp,sizeof(float)*Tar_Num_New);
		
			delete[]  Tar_Az_temp; Tar_Az_temp = NULL;
			delete[]  Tar_Ra_temp; Tar_Ra_temp = NULL;
			delete[]  Tar_Amp_temp; Tar_Amp_temp = NULL;
			delete[]  Tar_Ph_temp; Tar_Ph_temp = NULL;

			return Tar_Num_New;

		}

		int NewGYAimInfo::Point_Trace_1D(long *Tar_Ra,float *Tar_Amp, float *Tar_SNR,long Tar_Num,int win_ra)
		{
			// 点迹凝聚 OK 2021/06/06 CHECK dev by Yunhua-Luo 

			long   m,n,k,Tar_Num_New;
			long   *Tar_Ra_temp   =new  long[Tar_Num];
			float  *Tar_Amp_temp=new  float[Tar_Num];  
			float  *Tar_SNR_temp =new  float[Tar_Num];  
			k=0;

			for(m=0;m<Tar_Num;m++)
			{
				if(Tar_Amp[m]>0)			
				for(n=0;n<Tar_Num;n++) 
					Tar_Amp[m]*=float(!((abs(Tar_Ra[m]-Tar_Ra[n])<win_ra)&&(Tar_Amp[n]>Tar_Amp[m])&&(m!=n)));

				if(Tar_Amp[m]>0) 
				{	
					Tar_Ra_temp[k] = Tar_Ra[m];
					Tar_Amp_temp[k]= Tar_Amp[m];
					Tar_SNR_temp[k] = Tar_SNR[m];
					k++;
				}
			}

			Tar_Num_New=k;
			
			memcpy(Tar_Ra,   Tar_Ra_temp,    sizeof(long)*Tar_Num_New);
			memcpy(Tar_Amp,Tar_Amp_temp,sizeof(float)*Tar_Num_New);
			memcpy(Tar_SNR,Tar_SNR_temp,sizeof(float)*Tar_Num_New);

			delete[]  Tar_Ra_temp; Tar_Ra_temp = NULL;
			delete[]  Tar_Amp_temp; Tar_Amp_temp = NULL;
			delete[]  Tar_SNR_temp; Tar_SNR_temp = NULL;

			return Tar_Num_New;

		}

		int NewGYAimInfo::Point_SortByAmpl(long *Tar_Az, long *Tar_Ra, float *Tar_Amp, float *Tar_ph, long check_tarnum, int MaxTarNum)
		{
			long   *Tar_Az_temp = new long[check_tarnum];
			long   *Tar_Ra_temp = new long[check_tarnum];
			float  *Tar_Amp_temp = new float[check_tarnum];
			float  *Tar_Ph_temp = new float[check_tarnum];

			memcpy(Tar_Az_temp, Tar_Az, sizeof(long)*check_tarnum);
			memcpy(Tar_Ra_temp, Tar_Ra, sizeof(long)*check_tarnum);
			memcpy(Tar_Amp_temp, Tar_Amp, sizeof(float)*check_tarnum);
			memcpy(Tar_Ph_temp, Tar_ph, sizeof(float)*check_tarnum);

			__int64 *index = new __int64[check_tarnum];
			BubbleSort(Tar_Amp, check_tarnum, index);		// 按照能量大小排序
													 
			check_tarnum = MaxTarNum;
			for (int h = 0; h<check_tarnum; h++)
			{
				Tar_Az[h] = Tar_Az_temp[index[h]];
				Tar_Ra[h] = Tar_Ra_temp[index[h]];
				Tar_Amp[h] = Tar_Amp_temp[index[h]];
				Tar_ph[h] = Tar_Ph_temp[index[h]];
			}

			delete[] index; index = NULL;
			delete[] Tar_Az_temp; Tar_Az_temp = NULL;
			delete[] Tar_Ra_temp; Tar_Ra_temp = NULL;
			delete[] Tar_Amp_temp; Tar_Amp_temp = NULL;
			delete[] Tar_Ph_temp; Tar_Ph_temp = NULL;

			return check_tarnum;

		}


		template <class T>
		void NewGYAimInfo::median_phase_filter(T* phase,long len, int win_len)
		{// median phase filter for real-time WAS-GMTI mode
		 // dev by Yunhua-Luo @ 2017/05/08

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

		float NewGYAimInfo::Bit16ToDot(unsigned __int16 data)
		{// 16bit Verilog数转为二进制小数

			if(((data >> 15) & 0x01) == 1) 
			{
				data = ~(data & 0x7FFF);
			}

			char HighPart = (data >> 13) & 0x03;
			
			double LowPart = (data & 0x1FFF)/8192.0;

			float val = HighPart+LowPart;

			return val;
		}

        template <class T>
        void NewGYAimInfo::BubbleSort(T *pdata,__int64 N,__int64 *pIndex)
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

		void NewGYAimInfo::InfoTxtOut(char *file, float *DotInfo,int aim_num)
		{
		
			FILE *FL;
			FL = fopen(file, "at+");
			if (FL == NULL) return;

			sarImageInfo impar(head);
			muInfo  mu(head);
			fprintf(FL, "\n\n*************  WAS-MTI Dot Info ***********************\n");
			fprintf(FL,"日期(年.月.日):%04.0f.%02.0f.%02.0f\n",mu.date_year,mu.date_month,mu.date_day);
			fprintf(FL,"时间(时-分-秒-毫秒):%02.0f-%02.0f-%02.0f-%03.0f\n",mu.time_hour,mu.time_minutes,mu.time_second,mu.time_m_second);
			fprintf(FL,"飞机经度(度):%.7f\n",mu.plane_longitude);
			fprintf(FL,"飞机纬度(度):%.7f\n",mu.plane_latitude);
			fprintf(FL,"飞机海拔高度(米):%.3f\n",mu.plane_height);
			fprintf(FL,"机下点海拔高度(米):%.3f\n",mu.aimH);

			fprintf(FL, "最近斜距(米):%.2f\n", impar.slope_nearest);
			fprintf(FL, "最远斜距(米):%.2f\n", impar.slope_far);
			fprintf(FL, "场景中心斜距(米):%.2f\n", impar.slope_sen_center);

			fprintf(FL, "最近地距(米):%.2f\n", impar.ground_nearest);
			fprintf(FL, "最远地距(米):%.2f\n", impar.ground_far);

			fprintf(FL, "方位扫描范围(度):%.2f\n", 2*scanScope);
			fprintf(FL,"俯仰波束实际指向角(度):%.3f\n",mu.rAngle_pt);
			fprintf(FL,"飞机航向角(度):%.7f\n",mu.plane_direction_angle);
			fprintf(FL,"飞机俯仰角(度):%.7f\n",mu.plane_dive_angle);
			fprintf(FL,"飞机横滚角(度):%.7f\n",mu.plane_hor_angle);
			fprintf(FL,"飞机偏航角(度):%.7f\n",mu.plane_departure_angle);
			fprintf(FL,"地速(米/秒):%.4f\n\n",mu.plane_ground_v);

			fprintf(FL, "点迹个数: %d\n", aim_num);
			for (int i = 0; i<aim_num; i++)
			{
				fprintf(FL, "\n点迹编号：%04d\n", (i + 1));
				fprintf(FL, "当前扫描周期：%.7f\n", dotInfo[DOT_INFO_ITEM*i]);
				fprintf(FL, "当前波位号：%.7f\n", dotInfo[DOT_INFO_ITEM*i+1]);
				fprintf(FL, "点迹纬度(度)：%.7f\n", dotInfo[DOT_INFO_ITEM*i+2]);
				fprintf(FL, "点迹经度(度)：%.7f\n", dotInfo[DOT_INFO_ITEM*i+3]);
				fprintf(FL, "点迹高度(米)：%.3f\n", dotInfo[DOT_INFO_ITEM*i+4]);
				fprintf(FL, "径向速度(米/秒)：%.3f\n", dotInfo[DOT_INFO_ITEM*i+5]);
				fprintf(FL, "点迹强度    ：%.3f\n", dotInfo[DOT_INFO_ITEM*i+6]);
				fprintf(FL, "点迹SNR(dB)：%.3f\n", dotInfo[DOT_INFO_ITEM*i+7]);
			}
			fclose(FL);
		}

		int NewGYAimInfo::SettingPara(float &snr_th)
		{
			#define PARA_NUM 32
			char para_str[PARA_NUM][256];
			
			char *file = "DllConfig.ini";

			int       h = 0;
			FILE      *info_file_p;
			if ((info_file_p = fopen(file, "r")) != NULL)
			{
				for (h = 0; h<PARA_NUM; h++) { if (fscanf(info_file_p, "%s", para_str[h]) != EOF);  else  break; }
				fclose(info_file_p);
			}
			else
			{
				printf("para_file %s open fail\n", file);
				return 0;
			}

			int i = 0;
			while (i<PARA_NUM)
			{
				/////////// process mode parameter /////////////
				if (!strcmp(para_str[i], "SNR_Threshold:"))			  snr_th = atof(para_str[i + 1]);
				i++;
			}

		}

		int NewGYAimInfo::WASmti_locating_v2020(double *tar_info, long tar_num, sys_par par)
		{
			// WAS-MTI target locating function dev by Yunhua-Luo @ 2021/07/12 OK

			Tarlocate rd_loc;
			static long burst_cnt = 0;
			long h,m,j,k,l;
			__int64 B_Na = __int64(par.B_Na);	
			double R_near = par.R_near;			
			double prf = par.prf;  
			if(prf>8000 || prf < 0) prf = 1181.0;
			double az_vel = par.az_vel;			// 地速

			double scan_ang = par.scan_ang*PI/180.0; // 弧度 波束扫描角
			double elev_ang  = par.elev_ang*PI/180.0; // 弧度 俯仰角
			double yaw_ang  = par.yaw_ang*PI/180.0;  // 弧度 载机偏航角

			double lati0     = par.lati;					//载机纬度
			double logn0   = par.logn;				//载机经度
			double fly_ang = par.fly_ang;			// 载机航向角
			double Fs = par.Fs;							// 采样率
			double href = par.href;					//载机海拔
			double D = par.D;							// 天线长度
			double lookside = par.lookside;		//侧视方向 左--1，右 -- -1; -136 right 
			double lamda = C/double(par.fc);
			double Br = par.Br;
			double dr = C/2/Fs;
			double df = prf/B_Na;
			double tar_h = par.tar_height;		// 改为了dB，注意量化单位
			long Nr = par.Nr;
			int wave_no  = par.wave_no;
			int frame_no = par.frame_no;
			double e_v   = par.e_v;
			double n_v   = par.n_v;
			double u_v   = par.u_v;
			double fdc_est = par.fdc_est; // 对陆地的多普勒中心待改进

			float Target_MaxNum = 60; // 超过该数值则目标数认为是无效的；

			// 陆地目标凝聚参数
			unsigned __int8 WAS_WIN_RG = 64; 
			unsigned __int8 WAS_WIN_AZ = 32;

			// added by Luo 2021/07/08
			float vrbin = prf / B_Na*lamda / 2; // m/s
			WAS_WIN_AZ = int(B_Na / prf * 4 / vrbin/4.0+0.5)*4;
			float max_tarsize = 25;// 最大目标大小25m
			WAS_WIN_RG = int(max_tarsize / dr / 4.0) * 4;

	
			if(par.is_sea)
			{
				max_tarsize = 200;// 最大目标大小200m
				WAS_WIN_RG = int(max_tarsize / dr / 4.0) * 2;
				WAS_WIN_AZ = 32; 
				Target_MaxNum = 30;
				fdc_est = 0;
			}

			int win_ra = WAS_WIN_RG;
			int win_az = WAS_WIN_AZ;
			float min_vr_was = 4.0;      // can be adjust;
			double fd_amp = 2.0*az_vel*sin(scan_ang)*sin(fabs(elev_ang))/lamda; // added - by luo OK 
			int amb_num = round((fd_amp-fdc_est)/prf); 
			float fdc = fdc_est+prf*amb_num; 

			float *fa = new float[B_Na];
			for(h=0;h<B_Na;h++) fa[h] = fdc+h*prf/double(B_Na)-prf/2;
	
			// step 1. ---- data prepraring ------
			long  *Tar_Az     = new long[tar_num];
			long  *Tar_Ra     = new long[tar_num];
			float  *Tar_SNR  = new float[tar_num];
			float  *Tar_Amp  = new float[tar_num];
			float  *Tar_Ph      = new float[tar_num];
			float  *Tar_Wid   = new float[tar_num];

			float snr_th = 20;			// 添加参数可控
			SettingPara(snr_th);
			long cnt = 0;
			for(h=0; h<tar_num; h++)
			{
					if(tar_info[INFO_NUM*h+5] < snr_th || tar_info[INFO_NUM*h + 3] > B_Na || tar_info[INFO_NUM*h + 3] < 0) continue;  

					Tar_Amp[cnt] = tar_info[INFO_NUM*h + 0];
					Tar_Ra[cnt]  = long(tar_info[INFO_NUM*h+2]);		// range loc
					Tar_Az[cnt]  = long(tar_info[INFO_NUM*h+3]);
					Tar_Ph[cnt] = tar_info[INFO_NUM*h + 4];
					Tar_SNR[cnt] = tar_info[INFO_NUM*h+5];
					Tar_Wid[cnt] = tar_info[INFO_NUM*h+6];

					cnt++;
			}
			tar_num = cnt;

			__int64 check_tarnum = 0;

			if(par.is_sea)			// 对海模式采用非相参积累模式
			{
					// ************* 一维点迹凝聚  ******************
					check_tarnum = Point_Trace_1D(Tar_Ra,Tar_Amp,Tar_SNR,tar_num,win_ra);//点轨迹凝聚 OK
					if(check_tarnum<=0 || check_tarnum >= Target_MaxNum) return 0;

			}else  // 对地模式PD模式检测结果
			{
				 // ************* 二维点迹凝聚  ******************
					check_tarnum = Point_Trace(Tar_Az,Tar_Ra,Tar_Amp, Tar_Ph, tar_num,win_ra, win_az);
					if(check_tarnum<=0) return 0;

					if (check_tarnum >= Target_MaxNum)
					check_tarnum =  Point_SortByAmpl(Tar_Az, Tar_Ra, Tar_Amp, Tar_Ph, check_tarnum, Target_MaxNum);
			}

			/*
			FILE *fww;
			fww = fopen("d:\\testwas.dat", "wb");
			fwrite(Tar_Ra, check_tarnum, sizeof(long), fww);
			fwrite(Tar_Az, check_tarnum, sizeof(long), fww);
			fwrite(Tar_Amp, check_tarnum, sizeof(float), fww);
			fwrite(Tar_Ph, check_tarnum, sizeof(float), fww);
			fclose(fww);
			*/

			// ********** 求取目标干涉相位并完成定位 **********
			double yaw_ang_res = 0.0*PI/180.0;
			double PT,Rg,X_JD_t,Y_WD_t,lati,logn;
			double tar_vr,tar_r0,tar_fdc,tar_azimuth,tar_ampl,phi_t;
			long tar_validnum = 0;

			R_near += 150.0;  // Added for error 2021/06/06
			memset(dotInfo,0,DOT_INFO_ITEM*1024);
			for(h=0;h<check_tarnum;h++) 
			{
				PT = Tar_Ph[h]-(par.ch2_ph-par.ch1_ph)*PI/180.0;	 // Modified by luo 2021/01/24
				// 方位测角
				
				if(par.is_sea) // 对海版本采用实孔径进行测角，有误差！！！
				{
					tar_fdc = fd_amp;	// 测向太粗糙了！！
					tar_vr  = 0;				// 靠近航线为负，否则为正 按照六所要求；

				}else // 对陆地版本采用干涉相位进行测角，有误差！！！
				{
					// --- 如果中值相位合理，需要根据中值相位进行判读来进行定位 ---
					double psi = acos(sin(scan_ang)*sin(fabs(elev_ang)));
					double PT_cal = 2*PI*D*cos(psi)/lamda;
					double n_2PI = round((PT_cal-PT)/2/PI)*2*PI;

					double Elevation_angle = acos(href/(R_near+Tar_Ra[h]*dr));
					double PT1 = PT+n_2PI;
					double tar_psi = acos(PT1*lamda/2/PI/D);
					double tar_azi = asin(cos(tar_psi)/sin(Elevation_angle));

					//if(abs(abs(tar_azi*180.0/PI)-abs(scan_ang*180.0/PI))>3) continue;
					double angle_v = PI/2-tar_azi-lookside*yaw_ang_res;	   
					tar_fdc = 2.0*az_vel*cos(angle_v)*sin(Elevation_angle)/lamda;
					//tar_fdc = az_vel/(PI*D)*PT+fd_amp;  // simple way to locating 
					tar_vr  = -(tar_fdc - fa[Tar_Az[h]])*lamda/2.0;		// 靠近航线为负，否则为正 按照六所要求；
				}

				tar_r0  = R_near + Tar_Ra[h]*dr;				// 目标斜距			
				phi_t   = acos(href/tar_r0);					// 目标对应的俯仰角 单位幅度
				tar_azimuth = asin(tar_fdc*lamda/2/az_vel/sin(phi_t)); //目标方位角
				tar_ampl = Tar_Amp[h];

				// 运动目标定位 
				if(!WAS_RD_LOC_WAY)
				{
					Rg = sqrt(tar_r0*tar_r0-href*href);	
					X_JD_t = -Rg*lookside*cos((-tar_azimuth+fly_ang-yaw_ang_res)*lookside); // fly_ang-yaw_ang == true_fly_ang
					Y_WD_t = Rg*sin((-tar_azimuth+fly_ang-yaw_ang_res)*lookside);			// notice tar_azimuth +-  
				
					lati = lati0 + double(Y_WD_t)/double(111693.1);
					logn = logn0 + double(X_JD_t)/double(111693.1)/cos(lati0*PI/180);  // notice
				
				}else
				{
					// R-D locating 
					rd_par_input par_loc;
					double latlogn_rd[2],latlogn_geo[2];

					par_loc.fdc = tar_fdc;
					par_loc.href = href;
					par_loc.h_gnd = tar_h; 
					par_loc.lambda = lamda;
					par_loc.lati = lati0; // should be improve with new input 
					par_loc.logn = logn0; 
					par_loc.lookside = lookside;
					par_loc.r0 = tar_r0;
					par_loc.ve = e_v;	//
					par_loc.vn = n_v;	//
					par_loc.vu = u_v;		//u_v[Burst_Loc[n]+loc];

					rd_loc.RD_locating(par_loc,latlogn_rd,latlogn_geo);

					lati = latlogn_rd[0];
					logn = latlogn_rd[1];
				}
				
				// ************output results **************
				dotInfo[DOT_INFO_ITEM*tar_validnum]	  = frame_no;
				dotInfo[DOT_INFO_ITEM*tar_validnum+1] = wave_no;
				dotInfo[DOT_INFO_ITEM*tar_validnum+2] = lati;
				dotInfo[DOT_INFO_ITEM*tar_validnum+3] = logn;
				dotInfo[DOT_INFO_ITEM*tar_validnum+4] = tar_h;
				dotInfo[DOT_INFO_ITEM*tar_validnum+5] = tar_vr;
				dotInfo[DOT_INFO_ITEM*tar_validnum+6] = tar_ampl;
				dotInfo[DOT_INFO_ITEM*tar_validnum+7] = Tar_SNR[h];
				dotInfo[DOT_INFO_ITEM*tar_validnum+8] = Tar_Wid[h];
				dotInfo[DOT_INFO_ITEM*tar_validnum+9] = par.ts;
				dotInfo[DOT_INFO_ITEM*tar_validnum + 10] = tar_r0;
				dotInfo[DOT_INFO_ITEM*tar_validnum+11] = tar_azimuth;  // 弧度

				dotInfo[DOT_INFO_ITEM*tar_validnum + 12] = lati0;  // 飞机位置
				dotInfo[DOT_INFO_ITEM*tar_validnum + 13] = logn0;  
				dotInfo[DOT_INFO_ITEM*tar_validnum + 14] = fly_ang; 
				dotInfo[DOT_INFO_ITEM*tar_validnum + 15] = scan_ang*180.0/PI;  

				tar_validnum++;

			}

			// ****** output for *********
			/*
			InfoTxtOut("dzs_dotinfo.txt", dotInfo,tar_validnum);

			FILE *fw;
			fw = fopen("dzs_dotdata.dat", "ab");
			fwrite(dotInfo, sizeof(float), tar_validnum*DOT_INFO_ITEM, fw);
			fclose(fw);*/

			delete [] Tar_SNR; Tar_SNR = NULL;
			delete [] Tar_Ra;  Tar_Ra  = NULL;
			delete [] Tar_Amp; Tar_Amp = NULL;
			delete [] Tar_Ph; Tar_Ph = NULL;
			delete [] Tar_Az; Tar_Az = NULL;
			delete []  fa; fa = NULL;
			delete [] Tar_Wid; Tar_Wid = NULL;

			return tar_validnum;

			// OK check by Yunhua-Luo V4.0 2021/07/14
		}

		NewGYLineInfo::NewGYLineInfo(UINT8 *arIn)
		{
			memcpy(head,arIn,512);
			ar=arIn;
			linePathHeader.gyPathPt=NULL;
			linePathHeader.pathNum=0;
			CurScanAngle = *((__int16 *)(&ar[32]))*0.01;
			if(abs(CurScanAngle)>60)   CurScanAngle = 0;

			muInfo mu(arIn);
			plane_lati = mu.plane_latitude;
			plane_logn = mu.plane_longitude;
			
		}
		
		NewGYLineInfo::~NewGYLineInfo()
		{

		}

		int NewGYLineInfo::SettingTrackPara(NingjuPara & njPa, TrackPara & tkPa, KalManPara & KmPa)
		{
			#define PARA_NUM 128
			char *file = "DllConfig.ini";

			int       h = 0;
			FILE      *info_file_p;
			long      i;

			char para_str[PARA_NUM][256];

			i = 0;
			njPa.Xstep = 1000;
			njPa.Ystep = 1000;
			njPa.Zstep = 1000;

			njPa.Awidth = 0.03124/0.768*0.731*180/PI/cos(CurScanAngle*PI/180.0)/2;
			njPa.Rwidth = 200;
			njPa.Ewidth = 0;

			tkPa.Vmax = 300;
			tkPa.Vmin = 3;

			tkPa.m_tmp = 2;  // M/N准则判断是否转为确认航迹
			tkPa.n_tmp = 3;

			tkPa.EchoGateSize = tkPa.Vmax*tkPa.T;
			tkPa.miss_cnt_tmp = 4;    //连续预测错误计数则删除航迹
			tkPa.miss_cnt_trust = 3;  //达到缓存确认航迹点数最大值则删除航迹

			KmPa.Me = 300.0;
			KmPa.Mr = 300.0;
			KmPa.Ma = 300.0;
			KmPa.Qs = 1.0;
			KmPa.deltaA = 30.0;

			if ((info_file_p = fopen(file, "r")) != NULL)
			{
				for (h = 0; h<PARA_NUM; h++) { if (fscanf(info_file_p, "%s", para_str[h]) != EOF);  else  break; }
				fclose(info_file_p);
			}
			else
			{
				printf("para_file %s open fail\n", file);
				return 0;
			}


			while (i<PARA_NUM)
			{
				/////////// process mode parameter /////////////
				//if (!strcmp(para_str[i], "nj_xwin:"))			  njPa.Xstep = atof(para_str[i + 1]);
				//if (!strcmp(para_str[i], "nj_ywin:"))			  njPa.Ystep = atof(para_str[i + 1]);
				//if (!strcmp(para_str[i], "nj_zwin:"))			  njPa.Zstep = atof(para_str[i + 1]);

				if (!strcmp(para_str[i], "nj_Rwin:"))			  njPa.Rwidth = atof(para_str[i + 1]);
				if (!strcmp(para_str[i], "nj_Awin:"))			  njPa.Awidth = atof(para_str[i + 1]);
				if (!strcmp(para_str[i], "nj_Ewin:"))			  njPa.Ewidth = atof(para_str[i + 1]);

				if (!strcmp(para_str[i], "bomen_vmax:"))          tkPa.Vmax = atof(para_str[i + 1]);
				if (!strcmp(para_str[i], "bomen_vmin:"))          tkPa.Vmin = atof(para_str[i + 1]);
				if (!strcmp(para_str[i], "bomen_size:"))          tkPa.EchoGateSize = atof(para_str[i + 1]);

				if (!strcmp(para_str[i], "miss_frame_tmp:"))	  tkPa.miss_cnt_tmp = atoi(para_str[i + 1]);
				if (!strcmp(para_str[i], "miss_frame_trust:"))    tkPa.miss_cnt_trust = atoi(para_str[i + 1]);

				if (!strcmp(para_str[i], "NoiseMatrixRg:"))       KmPa.Mr = atof(para_str[i + 1]);
				if (!strcmp(para_str[i], "NoiseMatrixAz:"))       KmPa.Ma = atof(para_str[i + 1]);

				if (!strcmp(para_str[i], "NoiseSpetrumPowerDensity:"))     KmPa.Qs = atof(para_str[i + 1]);
				if (!strcmp(para_str[i], "TrackDir_thresholds:"))          KmPa.deltaA = atof(para_str[i + 1]);

				i++;
			}

		}

		void NewGYLineInfo::InfoTxtOut(char *file, float *TrkInfo,int trk_num,int frame_no)
		{
			FILE *FL;
			FL = fopen(file, "at+");
			if (FL == NULL) return;

			sarImageInfo impar(head);
			muInfo  mu(head);
		
			fprintf(FL, "\n\n*************  WAS-MTI Trk Info ***********************\n");
			fprintf(FL,"日期(年.月.日):%04.0f.%02.0f.%02.0f\n",mu.date_year,mu.date_month,mu.date_day);
			fprintf(FL,"时间(时-分-秒-毫秒):%02.0f-%02.0f-%02.0f-%03.0f\n",mu.time_hour,mu.time_minutes,mu.time_second,mu.time_m_second);
			fprintf(FL,"飞机经度(度):%.7f\n",mu.plane_longitude);
			fprintf(FL,"飞机纬度(度):%.7f\n",mu.plane_latitude);
			fprintf(FL,"飞机海拔高度(米):%.3f\n",mu.plane_height);
			fprintf(FL,"机下点海拔高度(米):%.3f\n",mu.aimH);

			fprintf(FL, "最近斜距(米):%.2f\n", impar.slope_nearest);
			fprintf(FL, "最远斜距(米):%.2f\n", impar.slope_far);
			fprintf(FL, "场景中心斜距(米):%.2f\n", impar.slope_sen_center);

			fprintf(FL, "最近地距(米):%.2f\n", impar.ground_nearest);
			fprintf(FL, "最远地距(米):%.2f\n", impar.ground_far);

			fprintf(FL,"飞机航向角(度):%.7f\n",mu.plane_direction_angle);
			fprintf(FL,"飞机俯仰角(度):%.7f\n",mu.plane_dive_angle);
			fprintf(FL,"飞机横滚角(度):%.7f\n",mu.plane_hor_angle);
			fprintf(FL,"飞机偏航角(度):%.7f\n",mu.plane_departure_angle);
			fprintf(FL,"地速(米/秒):%.4f\n\n",mu.plane_ground_v);
			fprintf(FL, "俯仰波束实际指向角(度):%.3f\n", mu.rAngle_pt);
			fprintf(FL, "Cfar检测门限值(dB):%.3f\n", mu.cfar_th_dB);


			fprintf(FL, "航迹个数: %d\n", trk_num);
			for (int k = 0; k < trk_num; k++)
			{
				// ******** Write into txtfile ********
				fprintf(FL, "\n航迹批号：%.0f\n", TrkInfo[TRK_INFO_ITEM*k] );
				fprintf(FL, "目标纬度：%.7f\n", TrkInfo[TRK_INFO_ITEM*k+1]);
				fprintf(FL, "目标经度：%.7f\n", TrkInfo[TRK_INFO_ITEM*k+2]);
				fprintf(FL, "目标高度：%.3f\n", TrkInfo[TRK_INFO_ITEM*k+3]);
				fprintf(FL, "目标速度：%.3f\n", TrkInfo[TRK_INFO_ITEM*k+4]);
				fprintf(FL, "目标方向：%.4f\n", TrkInfo[TRK_INFO_ITEM*k+5]);
				fprintf(FL, "目标幅度：%.2f\n", TrkInfo[TRK_INFO_ITEM*k+6]);
				fprintf(FL, "新批标识：   0\n");
				fprintf(FL, "目标属性：   0\n");
			}
			fclose(FL);
		}

		int NewGYLineInfo::MT_relate_XL_V2020(float *aimInfo,long tar_num, int scan_no, float scan_cycle)
		{// 2021/06/23

			commonFuncs cmFuncs;
			bool flag = false;
			if (tar_num <= 0) return flag;

			float tar_ref_alt = aimInfo[4]; // 实际为检测门限
			static vector<RawPoint> g_rawpoint_buff_full;
			
			static int scan_no_prev = -2;
			int frame_no = aimInfo[0];
			RawPoint tmp = {};
			long loc = 0;
			for (long k = 0; k < tar_num; k++)
			{
				tmp.frame_no = aimInfo[DOT_INFO_ITEM*k];
				tmp.wave_cnt = aimInfo[DOT_INFO_ITEM*k+1];
				tmp.lati   =  aimInfo[DOT_INFO_ITEM*k + 2];
				tmp.logn =	aimInfo[DOT_INFO_ITEM*k+3];
				tmp.alt =		aimInfo[DOT_INFO_ITEM*k+4];

				tmp.In =		aimInfo[DOT_INFO_ITEM*k+6];
				tmp.Vr =		aimInfo[DOT_INFO_ITEM*k+5];
				tmp.ts =		aimInfo[DOT_INFO_ITEM*k+9];
				
				tmp.R = aimInfo[DOT_INFO_ITEM*k+10];
				tmp.A = aimInfo[DOT_INFO_ITEM*k+11]; // Added by Luo 
				g_rawpoint_buff_full.push_back(tmp);

			}

			if (scan_no == scan_no_prev + 1)  // 扫描周期
			{
				mtitrack_para par_mtrk;

				g_mtt_obj_full.tar_ref_alt = tar_ref_alt;
				g_mtt_obj_full.plane_lati = plane_lati;
				g_mtt_obj_full.plane_logn = plane_logn;

				SettingTrackPara(par_mtrk.njPa, par_mtrk.tkPa, par_mtrk.KmPa);

				par_mtrk.tkPa.T = scan_cycle;

				cmFuncs.logRecords("Track Relation with FrameNo : ",scan_no);

				g_mtt_obj_full.Input(&g_rawpoint_buff_full, par_mtrk.njPa, par_mtrk.tkPa, par_mtrk.KmPa);
				
				tk_out_pts = g_mtt_obj_full.Solution(); 

				g_rawpoint_buff_full.clear();
				
				TrkNum = tk_out_pts->size();
			
				cmFuncs.logRecords("Track Relation Num ...", TrkNum);
				
				memset(TrkInfo,0,sizeof(float)*1024*TRK_INFO_ITEM);
				for(int k=0; k<TrkNum; k++)
				{	
					if (tk_out_pts->at(k).track_no < 100) continue;
					if (k >= 1024) break;
					TrkInfo[TRK_INFO_ITEM*k]      = tk_out_pts->at(k).track_no;
					TrkInfo[TRK_INFO_ITEM*k+1]  = tk_out_pts->at(k).lati;
					TrkInfo[TRK_INFO_ITEM*k+2]  = tk_out_pts->at(k).longti;
					TrkInfo[TRK_INFO_ITEM*k+3]  = tk_out_pts->at(k).alt;
					TrkInfo[TRK_INFO_ITEM*k+4]  = tk_out_pts->at(k).Vr;
					TrkInfo[TRK_INFO_ITEM*k+5]  = tk_out_pts->at(k).Heading;
					TrkInfo[TRK_INFO_ITEM*k+6]  = tk_out_pts->at(k).In;
					TrkInfo[TRK_INFO_ITEM*k+7]  = tk_out_pts->at(k).HrrpWid;
					TrkInfo[TRK_INFO_ITEM*k+8]  = plane_lati;
					TrkInfo[TRK_INFO_ITEM*k+9]  = plane_logn;
					TrkInfo[TRK_INFO_ITEM*k + 10] = tk_out_pts->at(k).ts;
					TrkInfo[TRK_INFO_ITEM*k + 11] = frame_no;
				}

				// output track information 
				InfoTxtOut("dzs_trkinfo.txt", TrkInfo,TrkNum,scan_no);
				flag = true;
				FILE *fid = fopen("dzs_trkdata.dat","wb");
				fwrite(TrkInfo,sizeof(float),TRK_INFO_ITEM*TrkNum,fid);
				fclose(fid);

				FILE *fiw = fopen("xlrtmti_dzs_allshow_trkdata.dat", "ab");
				fwrite(TrkInfo, sizeof(float), TRK_INFO_ITEM * 1024, fiw);  // Modified by Robles 2022/5/30;
				fclose(fiw);
			
			}
			scan_no_prev = scan_no;

			return flag;
		}


		//  **************************************************************
		//  &&&&&&&&&&&&  WAS-GMTI 实现主类 &&&&&&&&&&&&&&&
		
		GY_GMTIModel::GY_GMTIModel(UINT8 *arIn, string str)
		{// 2021/07/12 OK

				commonFuncs cmFuncs;
				ar=arIn;
				rootPath=str;
				dataName=cmFuncs.getName(ar);
				sockM=NULL;
				sockfd=NULL;
				gyNewLinePt=NULL;
				gyNewAim=NULL;
				gy_Multicast_init();

		}

		GY_GMTIModel::~GY_GMTIModel()
		{
			if(sockfd!=NULL)
			{
				closesocket(sockfd);
			}
			
			if(sockM!=NULL)
			{
				closesocket(sockfd);
			}

			if(gyNewAim != NULL) 
			{
				delete gyNewAim;
				gyNewAim = NULL;
			}

			if(gyNewLinePt != NULL)
			{
				delete gyNewLinePt;
				gyNewLinePt=NULL;
			}

			WSACleanup();
		}
		
		int GY_GMTIModel::gy_Multicast_init()
		{// 2021/07/12 modified for UDP Zubo;
			
			commonFuncs cmFuncs;

			//int portNum = 9001;
			//char ipAr[64] = "224.0.1.100";

			int portNum = 8000;
			char ipAr[64] = "127.0.0.1";

			//获取组播IP
			std::ifstream fin("NetConfig.ini",std::ios::in);
			if(fin)
			{
				char portAr[8]={0};
				fin.getline(ipAr, sizeof(ipAr));
				fin.getline(portAr,sizeof(portAr));
				fin.close();

				// get port number 
				int lenPortAr=strlen(portAr);
				portNum=portAr[0]-'0';
				for(int i=1;i<lenPortAr;i++)
				{
					int iTemp=portAr[i]-'0';
					portNum=10*portNum+iTemp;
				}
				cmFuncs.logRecords("Load NetConfig.ini Params OK ! ", 1);

			}else
			{
					cmFuncs.logRecords("NetConfig.ini Not Find, Default Parameters ...",1);
			}
			
			if(WSAStartup(MAKEWORD(2,2),&wsaData)!=0)
			{
					cmFuncs.logRecords("WSA Startup Failed ...", 1);
					return -1;
			}
         
			sockfd=WSASocket(AF_INET,SOCK_DGRAM,0,NULL,
					0,WSA_FLAG_MULTIPOINT_C_LEAF|WSA_FLAG_MULTIPOINT_D_LEAF|WSA_FLAG_OVERLAPPED);
				
			if(sockfd==INVALID_SOCKET)
			{
				cmFuncs.logRecords("WSA Socket Failed ...", 1);
				 return -1;
			}   
			memset(&servaddr,0,sizeof(servaddr));  
			servaddr.sin_family = AF_INET;  
			servaddr.sin_addr.s_addr = inet_addr(ipAr);  // old version 
			//servaddr.sin_addr.s_addr = INADDR_BROADCAST;  // can not receiver ...Modified by luo V20200304

			servaddr.sin_port = htons(portNum);
			sockM=WSAJoinLeaf(sockfd,(SOCKADDR*)&servaddr,sizeof(servaddr),NULL,NULL,NULL,NULL,JL_BOTH);
			if(sockM==INVALID_SOCKET)
			{
				cmFuncs.logRecords("WSA JointLeaf Failed ...", 1);
				return -1;
			} 
			return 0;		

		}


		// ********** WAS-GMTI AIM GENERATE *************
		int GY_GMTIModel::Generate_WasMTIAim()
		{// check 2021/07/12  V1

			// genereate WAS-GMTI data 
			if(gyNewAim == NULL) gyNewAim = new NewGYAimInfo(ar);
			
			aimNum = gyNewAim->aimsNum;

			// output Aiminfo to EMAP 
			outPut_NewGY_DatInfo();

			return 1;

		}

		int GY_GMTIModel::outPut_NewGY_DatInfo()
		{// 2020/11/25 check ok

			commonFuncs cmFuncs;
			
			int md=cmFuncs.getSarModel(ar);
			static int aScanMdAngle=0;

			// static string dataName=cmFuncs.getDataName(ar);
			string dataName = cmFuncs.getDataName_luo(ar); // added by luo 2016/4/1

			if(md!=4)
			{
				return ERROR_MODEL;
			}

			string gyDatInfoName=rootPath;

			gyDatInfoName.append(dataName+".dat");

			taskInfo task(ar);

			muInfo mu(ar);

			sarImageInfo imgPars(ar);

			GYParsInfo GYPars(ar);

			if(gyNewAim == NULL) return -1;
			
			UINT16 aimNms=gyNewAim->aimsNum;
			
			//用于发送  /***gySend1--gySend/
			char * sendFlName="gySend.dat";
			//FILE * FLSend=fopen(sendFlName,"w");
			FILE * FLSend=fopen(sendFlName,"wb");
			FILE * FL=fopen(gyDatInfoName.c_str(),"ab");

			if(FLSend==NULL || FL == NULL) return ERROR_MODEL;

			//char * bufferSend=new char[];
			//数据包信息
			UINT32 *packetInfo=new UINT32[3];
			packetInfo[0]=0X55AA55AA;
			fwrite(&(packetInfo[0]),sizeof(UINT32),1,FL);
			fwrite(&(packetInfo[0]),sizeof(UINT32),1,FLSend);
			
			packetInfo[1]=1;
			fwrite(&(packetInfo[1]),sizeof(UINT32),1,FL);
			fwrite(&(packetInfo[1]),sizeof(UINT32),1,FLSend);
			
			packetInfo[2]=216+aimNms*28;
            fwrite(&(packetInfo[2]),sizeof(UINT32),1,FL);
			fwrite(&(packetInfo[2]),sizeof(UINT32),1,FLSend);

			UINT16 taskCode=(UINT16)(task.missionCodes);
            //12
			fwrite(&taskCode,sizeof(UINT16),1,FL);	  //任务代号
			fwrite(&taskCode,sizeof(UINT16),1,FLSend);//任务代号
			//
			UINT8 planeType=7;

			fwrite(&planeType,sizeof(UINT8),1,FL);	  //飞行器类型
			fwrite(&planeType,sizeof(UINT8),1,FLSend);//飞行器类型

			UINT8 planePP=0;

			fwrite(&planePP,sizeof(UINT8),1,FL);	//飞机批号
			fwrite(&planePP,sizeof(UINT8),1,FLSend);//飞机批号

			UINT16 planeCode=(UINT16)(task.planeNum);
			
			fwrite(&planeCode,sizeof(UINT16),1,FL);//飞机号 
			fwrite(&planeCode,sizeof(UINT16),1,FLSend);//飞机号
            //18
			UINT8 picCompress=0;
			fwrite(&picCompress,sizeof(UINT8),1,FL);	 //图像压缩比
			fwrite(&picCompress,sizeof(UINT8),1,FLSend); //图像压缩比

			UINT8 transType=(UINT8)task.transType;
			fwrite(&transType,sizeof(UINT8),1,FL);//传输方式
			fwrite(&transType,sizeof(UINT8),1,FLSend);//传输方式

			UINT8 loadType=3;
			fwrite(&loadType,sizeof(UINT8),1,FL);
			fwrite(&loadType,sizeof(UINT8),1,FLSend);

			UINT32 loadCode=0;
			fwrite(&loadCode,sizeof(UINT32),1,FL);
			fwrite(&loadCode,sizeof(UINT32),1,FLSend);
            //25
			UINT16 powerOn=(UINT16)GYPars.powerOnTimes;
			fwrite(&powerOn,sizeof(UINT16),1,FL);
			fwrite(&powerOn,sizeof(UINT16),1,FLSend);

			UINT32 muCode=0;
			fwrite(&muCode,sizeof(UINT32),1,FL);
			fwrite(&muCode,sizeof(UINT32),1,FLSend);
            //31
			UINT16 year=(UINT16)mu.date_year;
			fwrite(&year,sizeof(UINT16),1,FL);
			fwrite(&year,sizeof(UINT16),1,FLSend);
             //33
			UINT8 month=(UINT8)mu.date_month;
			fwrite(&month,sizeof(UINT8),1,FL);
			fwrite(&month,sizeof(UINT8),1,FLSend);

			UINT8 day=(UINT8)mu.date_day;
			fwrite(&day,sizeof(UINT8),1,FL);
			fwrite(&day,sizeof(UINT8),1,FLSend);

			UINT8 hour=(UINT8)mu.time_hour;
			fwrite(&hour,sizeof(UINT8),1,FL);
			fwrite(&hour,sizeof(UINT8),1,FLSend);

			UINT8 minutes=(UINT8)mu.time_minutes;
			fwrite(&minutes,sizeof(UINT8),1,FL);
			fwrite(&minutes,sizeof(UINT8),1,FLSend);

			UINT8 seconds=(UINT8)mu.time_second;
			fwrite(&seconds,sizeof(UINT8),1,FL);
			fwrite(&seconds,sizeof(UINT8),1,FLSend);

			UINT16 mseconds=(UINT16)mu.time_m_second;
			fwrite(&mseconds,sizeof(UINT16),1,FL);
			fwrite(&mseconds,sizeof(UINT16),1,FLSend);
             //40
			double plane_Long=(double)mu.plane_longitude;
			fwrite(&plane_Long,sizeof(double),1,FL);
			fwrite(&plane_Long,sizeof(double),1,FLSend);

			double plane_Lat=(double)mu.plane_latitude;
			fwrite(&plane_Lat,sizeof(double),1,FL);//
			fwrite(&plane_Lat,sizeof(double),1,FLSend);//

			float * flt=new float[22];
			flt[0]=(float)mu.plane_height;
			fwrite(&(flt[0]),sizeof(float),1,FL);
			fwrite(&(flt[0]),sizeof(float),1,FLSend);

			flt[1]=(float)mu.plane_aim_height;
			fwrite(&(flt[1]),sizeof(float),1,FL);
			fwrite(&(flt[1]),sizeof(float),1,FLSend);

			flt[2]=(float)mu.plane_direction_angle;
			fwrite(&(flt[2]),sizeof(float),1,FL);
			fwrite(&(flt[2]),sizeof(float),1,FLSend);

			//航向角加速率
			flt[3]=(float)mu.plane_departure_angle;
			fwrite(&(flt[3]),sizeof(float),1,FL);
			fwrite(&(flt[3]),sizeof(float),1,FLSend);

			flt[4]=(float)mu.plane_departure_angle;
			fwrite(&(flt[4]),sizeof(float),1,FL);
			fwrite(&(flt[4]),sizeof(float),1,FLSend);

			//俯仰角
			flt[5]=(float)mu.plane_dive_angle;
			fwrite(&(flt[5]),sizeof(float),1,FL);
			fwrite(&(flt[5]),sizeof(float),1,FLSend);

			flt[6]=(float)mu.plane_dive_angle_V;
			fwrite(&(flt[6]),sizeof(float),1,FL);
			fwrite(&(flt[6]),sizeof(float),1,FLSend);

			flt[7]=(float)mu.plane_dive_angle_a;
			fwrite(&(flt[7]),sizeof(float),1,FL);
			fwrite(&(flt[7]),sizeof(float),1,FLSend);

			//横滚角
			flt[8]=(float)mu.plane_hor_angle;
			fwrite(&(flt[8]),sizeof(float),1,FL);
			fwrite(&(flt[8]),sizeof(float),1,FLSend);

            flt[9]=(float)mu.plane_hor_angle_v;
			fwrite(&(flt[9]),sizeof(float),1,FL);
			fwrite(&(flt[9]),sizeof(float),1,FLSend);

			flt[10]=(float)mu.plane_hor_angle_a;
			fwrite(&(flt[10]),sizeof(float),1,FL);
			fwrite(&(flt[10]),sizeof(float),1,FLSend);

			//
			flt[11]=(float)mu.plane_departure_angle;
			fwrite(&(flt[11]),sizeof(float),1,FL);
			fwrite(&(flt[11]),sizeof(float),1,FLSend);

			flt[12]=(float)mu.plane_de_flow_angle;
			fwrite(&(flt[12]),sizeof(float),1,FL);
			fwrite(&(flt[12]),sizeof(float),1,FLSend);

			flt[13]=(float)mu.plane_ground_v;
			fwrite(&(flt[13]),sizeof(float),1,FL);
			fwrite(&(flt[13]),sizeof(float),1,FLSend);

            flt[14]=(float)mu.plane_noair_v;
			fwrite(&(flt[14]),sizeof(float),1,FL);
			fwrite(&(flt[14]),sizeof(float),1,FLSend);

			flt[15]=(float)mu.plane_point_v;
			fwrite(&(flt[15]),sizeof(float),1,FL);
			fwrite(&(flt[15]),sizeof(float),1,FLSend);
			//
			flt[16]=(float)mu.plane_east_v;
			fwrite(&(flt[16]),sizeof(float),1,FL);
			fwrite(&(flt[16]),sizeof(float),1,FLSend);

			flt[17]=(float)mu.plane_north_v;
			fwrite(&(flt[17]),sizeof(float),1,FL);
			fwrite(&(flt[17]),sizeof(float),1,FLSend);

			flt[18]=(float)mu.plane_up_v;
			fwrite(&(flt[18]),sizeof(float),1,FL);
			fwrite(&(flt[18]),sizeof(float),1,FLSend);

			flt[19]=(float)mu.plane_east_a;
			fwrite(&(flt[19]),sizeof(float),1,FL);
			fwrite(&(flt[19]),sizeof(float),1,FLSend);

			flt[20]=(float)mu.plane_north_a;
			fwrite(&(flt[20]),sizeof(float),1,FL);
			fwrite(&(flt[20]),sizeof(float),1,FLSend);

			flt[21]=(float)mu.plane_up_a;
			fwrite(&(flt[21]),sizeof(float),1,FL);
			fwrite(&(flt[21]),sizeof(float),1,FLSend);

			if(flt!=NULL)
			{
				delete []flt;
				flt=NULL;
			}

			//天线帧编号
			UINT32 frameNum=(UINT32)GYPars.frameCodes;
			fwrite(&frameNum,sizeof(UINT32),1,FL);
			fwrite(&frameNum,sizeof(UINT32),1,FLSend);

			//波位数
			UINT16 waveNum= (UINT16)GYPars.waveNum;
			fwrite(&waveNum,sizeof(UINT16),1,FL);
			fwrite(&waveNum,sizeof(UINT16),1,FLSend);

			//波位号
			UINT16 waveCode=(UINT16)GYPars.waveCodes;
			fwrite(&waveCode,sizeof(UINT16),1,FL);
			fwrite(&waveCode,sizeof(UINT16),1,FLSend);

			//工作频段 侧视方式  工作模式 工作子模式
			UINT8 * FreqLS=new UINT8[4];
			FreqLS[0]=0;
			fwrite(&(FreqLS[0]),sizeof(UINT8),1,FL);
			fwrite(&(FreqLS[0]),sizeof(UINT8),1,FLSend);

			if(mu.rAngle<0) // 右侧视
			{
				FreqLS[1] = 1;
			}else           // 左侧视
			{
				FreqLS[1] = 0;
			}
			fwrite(&(FreqLS[1]),sizeof(UINT8),1,FL);
			fwrite(&(FreqLS[1]),sizeof(UINT8),1,FLSend);

			FreqLS[2]=0; // GMTI
			fwrite(&(FreqLS[2]),sizeof(UINT8),1,FL);
			fwrite(&(FreqLS[2]),sizeof(UINT8),1,FLSend);

			FreqLS[3]=0;// WAS
			fwrite(&(FreqLS[3]),sizeof(UINT8),1,FL);
			fwrite(&(FreqLS[3]),sizeof(UINT8),1,FLSend);

			UINT32 *fNDistance=new UINT32[2];

			fNDistance[1]=(UINT32)GYPars.RFar;
			fNDistance[0]=(UINT32)GYPars.RNear;

			fwrite(&(fNDistance[1]),sizeof(UINT32),1,FL);
			fwrite(&(fNDistance[0]),sizeof(UINT32),1,FL);

			fwrite(&(fNDistance[1]),sizeof(UINT32),1,FLSend);
			fwrite(&(fNDistance[0]),sizeof(UINT32),1,FLSend);

			if(fNDistance!=NULL)
			{
				delete []fNDistance;
				fNDistance=NULL;
			}

			float * scanPar=new float[8];

			//天线帧扫描中心角	
			float temp_angle = mu.plane_direction_angle-mu.plane_departure_angle-imgPars.look_Side*90;//OK
			if(temp_angle < 0) temp_angle += 360;

			scanPar[0] = (float)temp_angle;
			fwrite(&(scanPar[0]),sizeof(float),1,FL);
			fwrite(&(scanPar[0]),sizeof(float),1,FLSend);

			//天线帧扫描范围
			scanPar[1]=2*(float)GYPars.scanScope;
			fwrite(&(scanPar[1]),sizeof(float),1,FL);
			fwrite(&(scanPar[1]),sizeof(float),1,FLSend);

			//方位波束中心角
			scanPar[2]=(float)GYPars.azimuthCenterAngle;
			fwrite(&(scanPar[2]),sizeof(float),1,FL);
			fwrite(&(scanPar[2]),sizeof(float),1,FLSend);

			scanPar[3]=(float)imgPars.beam_horz_width;
			fwrite(&(scanPar[3]),sizeof(float),1,FL);
			fwrite(&(scanPar[3]),sizeof(float),1,FLSend);

			scanPar[4]=(int)imgPars.beam_horz_width;
			fwrite(&(scanPar[4]),sizeof(float),1,FL);
			fwrite(&(scanPar[4]),sizeof(float),1,FLSend);
			//俯仰波束中心角
			scanPar[5]=imgPars.R_angle_new;
			fwrite(&(scanPar[5]),sizeof(float),1,FL);
			fwrite(&(scanPar[5]),sizeof(float),1,FLSend);
			//俯仰波束宽度
			scanPar[6]=imgPars.beam_R_width;
			fwrite(&(scanPar[6]),sizeof(float),1,FL);
			fwrite(&(scanPar[6]),sizeof(float),1,FLSend);
			//俯仰向扫描步进
			scanPar[7]=0.0;
			fwrite(&(scanPar[7]),sizeof(float),1,FL);
			fwrite(&(scanPar[7]),sizeof(float),1,FLSend);

			if(scanPar!=NULL)
			{
				delete []scanPar;
				scanPar=NULL;
			}		

			UINT16 resPulseNum=(UINT16)GYPars.pulseResident;
			fwrite(&resPulseNum,sizeof(UINT16),1,FL);
			fwrite(&resPulseNum,sizeof(UINT16),1,FLSend);

			UINT16 resTime= UINT16(GYPars.pulseResident*1000/GYPars.prf);
			fwrite(&resTime,sizeof(UINT16),1,FL);
			fwrite(&resTime,sizeof(UINT16),1,FLSend);
		    //分辨率
			float resolv=imgPars.sarRes;
			fwrite(&resolv,sizeof(float),1,FL);
			fwrite(&resolv,sizeof(float),1,FLSend);

			UINT32 PRF=(UINT32)GYPars.prf;
			fwrite(&PRF,sizeof(UINT32),1,FL);
			fwrite(&PRF,sizeof(UINT32),1,FLSend);

			UINT16 nullPs=0;
			fwrite(&nullPs,sizeof(UINT16),1,FL);
			fwrite(&nullPs,sizeof(UINT16),1,FLSend);
			
			fwrite(&aimNum,sizeof(UINT16),1,FL);
			fwrite(&aimNum,sizeof(UINT16),1,FLSend);

			// ************output results **************
			__int64 h=0;
			while(h<aimNum)
			{
			   UINT32 pointCodes=(UINT32)h;
			   fwrite(&pointCodes,sizeof(UINT32),1,FL);
			   fwrite(&pointCodes,sizeof(UINT32),1,FLSend);
               //序号
			   double pointLong=dotInfo[DOT_INFO_ITEM*h+3];  //经度
			   fwrite(&pointLong,sizeof(double),1,FL);
			   fwrite(&pointLong,sizeof(double),1,FLSend);

			   double pointLat=dotInfo[DOT_INFO_ITEM*h+2];    //纬度
			   fwrite(&pointLat,sizeof(double),1,FL);
			   fwrite(&pointLat,sizeof(double),1,FLSend);

			   float pointHeight=dotInfo[DOT_INFO_ITEM*h+4]; //imgPars.aim_height; //modfied by luo 2020
			   fwrite(&pointHeight,sizeof(float),1,FL);
			   fwrite(&pointHeight,sizeof(float),1,FLSend);

			   float point_P_v=dotInfo[DOT_INFO_ITEM*h+5];
			   fwrite(&point_P_v,sizeof(float),1,FL);
			   fwrite(&point_P_v,sizeof(float),1,FLSend);
			   
			   h++;
			}

			UINT32 packetTail=0X0D0D0D0D;
			fwrite(&packetTail,sizeof(UINT32),1,FL);
			fwrite(&packetTail,sizeof(UINT32),1,FLSend);

			fclose(FL);
			fclose(FLSend);

			FLSend=fopen(sendFlName,"rb");
			if(FLSend==NULL)
			{
				return -1;
			}

			fseek(FLSend,0, SEEK_END);
			int file_size = ftell(FLSend);
			fseek(FLSend,0,SEEK_SET);

			char * buffer=new char[file_size];	
			fread(buffer,sizeof(char),file_size,FLSend);
			fclose(FLSend);
			
		    int resTemp=sendto(sockfd, buffer,file_size, 0,  
				(struct sockaddr *)&servaddr, sizeof(sockaddr));
			if(buffer!=NULL)
			{
				delete [] buffer;
				buffer=NULL;
			}

			return resTemp;
		}
		
		// ********* WAS-GMTI FORM TRACKS **************
		int GY_GMTIModel::Generate_WasMTITrack()
		{// check 2022/05/28 by Dr.Luo

			commonFuncs cmFuncs;
			string rootForGeoMap = rootPath;
		
			if(gyNewLinePt==NULL) gyNewLinePt = new NewGYLineInfo(ar);
		
			GYParsInfo gypar(ar);
			muInfo mupar(ar);
			static int frameCode = -2;
			if(gypar.waveCodes == 1 || frameCode == -2) // 调整输出参数对应的波位号
			memcpy(prev_ar,ar,sizeof(unsigned char)*512);  // save previous ar data 

			//cmFuncs.logRecords("Begin Track Relation ...",1);
			gyNewLinePt->plane_lati   = mupar.plane_latitude;
			gyNewLinePt->plane_logn = mupar.plane_longitude;

			bool flag = gyNewLinePt->MT_relate_XL_V2020(dotInfo,aimNum,gypar.frameCodes,gypar.scanCycle);
		
			if(flag)
			{
				trkNum = gyNewLinePt->TrkNum;
				cmFuncs.logRecords("Send Track Info to EMAP ...",double(gypar.frameCodes));
				outPut_NewGY_Line_DatInfo(prev_ar);  // send to e-map 
			}
			frameCode = gypar.frameCodes;

			return 1;		
		}

		int GY_GMTIModel::outPut_NewGY_Line_DatInfo(unsigned char *ar_old)
		{
			// 采用上一次的辅助数据进行参数输出 
            commonFuncs cmFuncs;
			int md=cmFuncs.getSarModel(ar_old);
			static int aScanMdAngle=0;
			
			if(md!=4)
			{
				return ERROR_MODEL;
			}
			if(gyNewLinePt == NULL) return -1;

			// par inti
			taskInfo task(ar_old);

			muInfo mu(ar_old);

			sarImageInfo imgPars(ar_old);

			GYParsInfo GYPars(ar_old);
			
			dataName = cmFuncs.getDataName_luo(ar_old);

			string gyDatInfoName=rootPath;
			gyDatInfoName.append(dataName+".dat");

			UINT16 aimNms= UINT16(gyNewLinePt->linePathHeader.pathNum);
			
			//用于发送
			char * sendFlName="gyLineSend.dat";
			FILE * FLSend=fopen(sendFlName,"wb");

			FILE * FL=fopen(gyDatInfoName.c_str(),"ab");
			if(FLSend==NULL || FL == NULL) return ERROR_MODEL;

			//数据包信息
			UINT32 *packetInfo=new UINT32[3];
			packetInfo[0]=0X55AA55AA;
			fwrite(&(packetInfo[0]),sizeof(UINT32),1,FL);
			fwrite(&(packetInfo[0]),sizeof(UINT32),1,FLSend);
			packetInfo[1]=2;
			fwrite(&(packetInfo[1]),sizeof(UINT32),1,FL);
			fwrite(&(packetInfo[1]),sizeof(UINT32),1,FLSend);
			packetInfo[2]=216+ trkNum*36;
            fwrite(&(packetInfo[2]),sizeof(UINT32),1,FL);
			fwrite(&(packetInfo[2]),sizeof(UINT32),1,FLSend);

			UINT16 taskCode=(UINT16)(task.missionCodes);
			fwrite(&taskCode,sizeof(UINT16),1,FL);//任务代号
			fwrite(&taskCode,sizeof(UINT16),1,FLSend);//任务代号
			//
			UINT8 planeType=7;

			fwrite(&planeType,sizeof(UINT8),1,FL);//飞行器类型
			fwrite(&planeType,sizeof(UINT8),1,FLSend);//飞行器类型

			UINT8 planePP=0;

			fwrite(&planePP,sizeof(UINT8),1,FL);//飞机批号
			fwrite(&planePP,sizeof(UINT8),1,FLSend);//飞机批号

			UINT16 planeCode=(UINT16)(task.planeNum);
			
			fwrite(&planeCode,sizeof(UINT16),1,FL);//飞机号 
			fwrite(&planeCode,sizeof(UINT16),1,FLSend);//飞机号
             //18

			UINT8 picCompress=0;
			fwrite(&picCompress,sizeof(UINT8),1,FL);	//图像压缩比
			fwrite(&picCompress,sizeof(UINT8),1,FLSend);//图像压缩比

			UINT8 transType=(UINT8)task.transType;
			fwrite(&transType,sizeof(UINT8),1,FL);//传输方式
			fwrite(&transType,sizeof(UINT8),1,FLSend);//传输方式

			UINT8 loadType=3;
			fwrite(&loadType,sizeof(UINT8),1,FL);
			fwrite(&loadType,sizeof(UINT8),1,FLSend);

			UINT32 loadCode=0;
			fwrite(&loadCode,sizeof(UINT32),1,FL);
			fwrite(&loadCode,sizeof(UINT32),1,FLSend);
            //25
			UINT16 powerOn=(UINT16)GYPars.powerOnTimes;
			fwrite(&powerOn,sizeof(UINT16),1,FL);
			fwrite(&powerOn,sizeof(UINT16),1,FLSend);

			UINT32 muCode=0;
			fwrite(&muCode,sizeof(UINT32),1,FL);
			fwrite(&muCode,sizeof(UINT32),1,FLSend);
            //31
			UINT16 year=(UINT16)mu.date_year;
			fwrite(&year,sizeof(UINT16),1,FL);
			fwrite(&year,sizeof(UINT16),1,FLSend);
             //33
			UINT8 month=(UINT8)mu.date_month;
			fwrite(&month,sizeof(UINT8),1,FL);
			fwrite(&month,sizeof(UINT8),1,FLSend);

			UINT8 day=(UINT8)mu.date_day;
			fwrite(&day,sizeof(UINT8),1,FL);
			fwrite(&day,sizeof(UINT8),1,FLSend);

			UINT8 hour=(UINT8)mu.time_hour;
			fwrite(&hour,sizeof(UINT8),1,FL);
			fwrite(&hour,sizeof(UINT8),1,FLSend);

			UINT8 minutes=(UINT8)mu.time_minutes;
			fwrite(&minutes,sizeof(UINT8),1,FL);
			fwrite(&minutes,sizeof(UINT8),1,FLSend);

			UINT8 seconds=(UINT8)mu.time_second;
			fwrite(&seconds,sizeof(UINT8),1,FL);
			fwrite(&seconds,sizeof(UINT8),1,FLSend);

			UINT16 mseconds=(UINT16)mu.time_m_second;
			fwrite(&mseconds,sizeof(UINT16),1,FL);
			fwrite(&mseconds,sizeof(UINT16),1,FLSend);
             //40
			double plane_Long=(double)mu.plane_longitude;
			fwrite(&plane_Long,sizeof(double),1,FL);
			fwrite(&plane_Long,sizeof(double),1,FLSend);

			double plane_Lat=(double)mu.plane_latitude;
			fwrite(&plane_Lat,sizeof(double),1,FL);//
			fwrite(&plane_Lat,sizeof(double),1,FLSend);//

			float * flt=new float[22];
             //56
			flt[0]=(float)mu.plane_height;
			fwrite(&(flt[0]),sizeof(float),1,FL);
			fwrite(&(flt[0]),sizeof(float),1,FLSend);

			flt[1]=(float)mu.plane_aim_height;
			fwrite(&(flt[1]),sizeof(float),1,FL);
			fwrite(&(flt[1]),sizeof(float),1,FLSend);

			flt[2]=(float)mu.plane_direction_angle;
			fwrite(&(flt[2]),sizeof(float),1,FL);
			fwrite(&(flt[2]),sizeof(float),1,FLSend);

			//航向角加速率
			flt[3]=(float)mu.plane_departure_angle;
			fwrite(&(flt[3]),sizeof(float),1,FL);
			fwrite(&(flt[3]),sizeof(float),1,FLSend);

			flt[4]=(float)mu.plane_departure_angle;
			fwrite(&(flt[4]),sizeof(float),1,FL);
			fwrite(&(flt[4]),sizeof(float),1,FLSend);

			//俯仰角
			flt[5]=(float)mu.plane_dive_angle;
			fwrite(&(flt[5]),sizeof(float),1,FL);
			fwrite(&(flt[5]),sizeof(float),1,FLSend);

			flt[6]=(float)mu.plane_dive_angle_V;
			fwrite(&(flt[6]),sizeof(float),1,FL);
			fwrite(&(flt[6]),sizeof(float),1,FLSend);

			flt[7]=(float)mu.plane_dive_angle_a;
			fwrite(&(flt[7]),sizeof(float),1,FL);
			fwrite(&(flt[7]),sizeof(float),1,FLSend);

			//横滚角
			flt[8]=(float)mu.plane_hor_angle;
			fwrite(&(flt[8]),sizeof(float),1,FL);
			fwrite(&(flt[8]),sizeof(float),1,FLSend);

            flt[9]=(float)mu.plane_hor_angle_v;
			fwrite(&(flt[9]),sizeof(float),1,FL);
			fwrite(&(flt[9]),sizeof(float),1,FLSend);

			flt[10]=(float)mu.plane_hor_angle_a;
			fwrite(&(flt[10]),sizeof(float),1,FL);
			fwrite(&(flt[10]),sizeof(float),1,FLSend);

			flt[11]=(float)mu.plane_departure_angle;
			fwrite(&(flt[11]),sizeof(float),1,FL);
			fwrite(&(flt[11]),sizeof(float),1,FLSend);

			flt[12]=(float)mu.plane_de_flow_angle;
			fwrite(&(flt[12]),sizeof(float),1,FL);
			fwrite(&(flt[12]),sizeof(float),1,FLSend);

			flt[13]=(float)mu.plane_ground_v;
			fwrite(&(flt[13]),sizeof(float),1,FL);
			fwrite(&(flt[13]),sizeof(float),1,FLSend);

            flt[14]=(float)mu.plane_noair_v;
			fwrite(&(flt[14]),sizeof(float),1,FL);
			fwrite(&(flt[14]),sizeof(float),1,FLSend);

			flt[15]=(float)mu.plane_point_v;
			fwrite(&(flt[15]),sizeof(float),1,FL);
			fwrite(&(flt[15]),sizeof(float),1,FLSend);
			//
			flt[16]=(float)mu.plane_east_v;
			fwrite(&(flt[16]),sizeof(float),1,FL);
			fwrite(&(flt[16]),sizeof(float),1,FLSend);

			flt[17]=(float)mu.plane_north_v;
			fwrite(&(flt[17]),sizeof(float),1,FL);
			fwrite(&(flt[17]),sizeof(float),1,FLSend);

			flt[18]=(float)mu.plane_up_v;
			fwrite(&(flt[18]),sizeof(float),1,FL);
			fwrite(&(flt[18]),sizeof(float),1,FLSend);

			flt[19]=(float)mu.plane_east_a;
			fwrite(&(flt[19]),sizeof(float),1,FL);
			fwrite(&(flt[19]),sizeof(float),1,FLSend);

			flt[20]=(float)mu.plane_north_a;
			fwrite(&(flt[20]),sizeof(float),1,FL);
			fwrite(&(flt[20]),sizeof(float),1,FLSend);

			flt[21]=(float)mu.plane_up_a;
			fwrite(&(flt[21]),sizeof(float),1,FL);
			fwrite(&(flt[21]),sizeof(float),1,FLSend);

			if(flt!=NULL)
			{
				delete []flt;
				flt=NULL;
			}
          
			//天线帧编号
			UINT32 frameNum=(UINT32)GYPars.frameCodes;
			fwrite(&frameNum,sizeof(UINT32),1,FL);
			fwrite(&frameNum,sizeof(UINT32),1,FLSend);

			//波位数
			UINT16 waveNum=(UINT16)GYPars.waveNum;
			fwrite(&waveNum,sizeof(UINT16),1,FL);
			fwrite(&waveNum,sizeof(UINT16),1,FLSend);

			//波位号
			UINT16 waveCode=(UINT16)GYPars.waveCodes;
			fwrite(&waveCode,sizeof(UINT16),1,FL);
			fwrite(&waveCode,sizeof(UINT16),1,FLSend);

			//工作频段 侧视方式  工作模式 工作子模式
			UINT8 * FreqLS=new UINT8[4];
			FreqLS[0]=0;
			fwrite(&(FreqLS[0]),sizeof(UINT8),1,FL);
			fwrite(&(FreqLS[0]),sizeof(UINT8),1,FLSend);

			if(mu.rAngle<0) // 右侧视
			{
				FreqLS[1] = 1;
			}else           // 左侧视
			{
				FreqLS[1] = 0;
			}
			fwrite(&(FreqLS[1]),sizeof(UINT8),1,FL);
			fwrite(&(FreqLS[1]),sizeof(UINT8),1,FLSend);

			FreqLS[2]=0;
			fwrite(&(FreqLS[2]),sizeof(UINT8),1,FL);
			fwrite(&(FreqLS[2]),sizeof(UINT8),1,FLSend);

			FreqLS[3]=0;
			fwrite(&(FreqLS[3]),sizeof(UINT8),1,FL);
			fwrite(&(FreqLS[3]),sizeof(UINT8),1,FLSend);

			//156
			UINT32 *fNDistance=new UINT32[2];

			fNDistance[1]=(UINT32 )GYPars.RFar;
			fNDistance[0]=(UINT32 )GYPars.RNear;//dbRNear

			fwrite(&(fNDistance[1]),sizeof(UINT32),1,FL);
			fwrite(&(fNDistance[0]),sizeof(UINT32),1,FL);

			fwrite(&(fNDistance[1]),sizeof(UINT32),1,FLSend);
			fwrite(&(fNDistance[0]),sizeof(UINT32),1,FLSend);
			//fwrite(fNDistance,sizeof(UINT32),2,FLSend);

			if(fNDistance!=NULL)
			{
				delete []fNDistance;
				fNDistance=NULL;
			}

			float * scanPar=new float[8];

			//天线帧扫描中心角		
			float temp_angle = mu.plane_direction_angle-mu.plane_departure_angle-imgPars.look_Side*90;	//OK
			if(temp_angle < 0) temp_angle += 360;

			scanPar[0] = (float)temp_angle;
			fwrite(&(scanPar[0]),sizeof(float),1,FL);
			fwrite(&(scanPar[0]),sizeof(float),1,FLSend);
			//天线帧扫描范围
			scanPar[1]=2*(float)GYPars.scanScope;
			fwrite(&(scanPar[1]),sizeof(float),1,FL);
			fwrite(&(scanPar[1]),sizeof(float),1,FLSend);

			//方位波束中心角
			scanPar[2]=(float)GYPars.azimuthCenterAngle;

			fwrite(&(scanPar[2]),sizeof(float),1,FL);
			fwrite(&(scanPar[2]),sizeof(float),1,FLSend);

			scanPar[3]=(float)imgPars.beam_horz_width;
			
			fwrite(&(scanPar[3]),sizeof(float),1,FL);
			fwrite(&(scanPar[3]),sizeof(float),1,FLSend);

			scanPar[4]=(int)imgPars.beam_horz_width;
			fwrite(&(scanPar[4]),sizeof(float),1,FL);
			fwrite(&(scanPar[4]),sizeof(float),1,FLSend);
			//俯仰波束中心角
			scanPar[5]=imgPars.R_angle_new;
			fwrite(&(scanPar[5]),sizeof(float),1,FL);
			fwrite(&(scanPar[5]),sizeof(float),1,FLSend);
			//俯仰波束宽度
			scanPar[6]=imgPars.beam_R_width;
			fwrite(&(scanPar[6]),sizeof(float),1,FL);
			fwrite(&(scanPar[6]),sizeof(float),1,FLSend);
			//俯仰向扫描步进
			scanPar[7]=0;
			fwrite(&(scanPar[7]),sizeof(float),1,FL);
			fwrite(&(scanPar[7]),sizeof(float),1,FLSend);
		
			if(scanPar!=NULL)
			{
				delete []scanPar;
				scanPar=NULL;
			}		

			UINT16 resPulseNum=(UINT16)GYPars.pulseResident;
			fwrite(&resPulseNum,sizeof(UINT16),1,FL);
			fwrite(&resPulseNum,sizeof(UINT16),1,FLSend);

			UINT16 resTime= UINT16(GYPars.pulseResident*1000/GYPars.prf);
			fwrite(&resTime,sizeof(UINT16),1,FL);
			fwrite(&resTime,sizeof(UINT16),1,FLSend);
		    //分辨率
			float resolv=imgPars.sarRes;
			fwrite(&resolv,sizeof(float),1,FL);
			fwrite(&resolv,sizeof(float),1,FLSend);

			UINT32 PRF=(UINT32)GYPars.prf;
			fwrite(&PRF,sizeof(UINT32),1,FL);
			fwrite(&PRF,sizeof(UINT32),1,FLSend);

			UINT16 nullPs=0;
			fwrite(&nullPs,sizeof(UINT16),1,FL);
			fwrite(&nullPs,sizeof(UINT16),1,FLSend);

			// ********* WAS-GMTI HJ OUTPUT V2021/06/08 ********
			UINT16 hj_num = (UINT16)trkNum;
			fwrite(&hj_num,sizeof(UINT16),1,FL);
			fwrite(&hj_num,sizeof(UINT16),1,FLSend);

			cmFuncs.logRecords("Output Track Line Num New ...", hj_num);

			for(int h=0; h<hj_num; h++)
			{

				  // cmFuncs.logRecords("TarNo : ", h);

				   UINT32 pointCodes=(UINT32)TrkInfo[TRK_INFO_ITEM*h] ;
				   fwrite(&pointCodes,sizeof(UINT32),1,FL);
				   fwrite(&pointCodes,sizeof(UINT32),1,FLSend);
				   //序号
				   double pointLong=TrkInfo[TRK_INFO_ITEM*h+2] ;  //经度
				   fwrite(&pointLong,sizeof(double),1,FL);
				   fwrite(&pointLong,sizeof(double),1,FLSend);

				   double pointLat=TrkInfo[TRK_INFO_ITEM*h+1] ;  //纬度
				   fwrite(&pointLat,sizeof(double),1,FL);
				   fwrite(&pointLat,sizeof(double),1,FLSend);

				   float pointHeight=TrkInfo[TRK_INFO_ITEM*h+3] ;
				   fwrite(&pointHeight,sizeof(float),1,FL);
				   fwrite(&pointHeight,sizeof(float),1,FLSend);

				   float point_P_v=TrkInfo[TRK_INFO_ITEM*h+4] ;
				   fwrite(&point_P_v,sizeof(float),1,FL);
				   fwrite(&point_P_v,sizeof(float),1,FLSend);

				   float point_direction=(float)TrkInfo[TRK_INFO_ITEM*h+5] ;
				   fwrite(&point_direction,sizeof(float),1,FL);
				   fwrite(&point_direction,sizeof(float),1,FLSend);

				   UINT8 If_New = 1; // (UINT8)TrkInfo[TRK_INFO_ITEM*h];
				   fwrite(&If_New,sizeof(UINT8),1,FL);
				   fwrite(&If_New,sizeof(UINT8),1,FLSend);

				   UINT8 aim_prop = 0; // (UINT8)TrkInfo[TRK_INFO_ITEM*h];
				   fwrite(&aim_prop,sizeof(UINT8),1,FL);
				   fwrite(&aim_prop,sizeof(UINT8),1,FLSend);

				   //预留 2021/06/08
				   UINT16 non_null = UINT16(TrkInfo[TRK_INFO_ITEM*h + 6]);  // 输出目标的In;
				   fwrite(&non_null,sizeof(UINT16),1,FL);
				   fwrite(&non_null,sizeof(UINT16),1,FLSend);

			}
			
			UINT32 packetTail=0X0D0D0D0D;
			fwrite(&packetTail,sizeof(UINT32),1,FL);
			fwrite(&packetTail,sizeof(UINT32),1,FLSend);

			fclose(FL);
			fclose(FLSend);

			FLSend=fopen(sendFlName,"rb");

			if(FLSend==NULL)
			{
				return -1;
			}

			fseek(FLSend,0, SEEK_END);
			int file_size = ftell(FLSend);
			fseek(FLSend,0,SEEK_SET);

			char * buffer=new char[file_size];			
			fread(buffer,sizeof(char),file_size,FLSend);
			fclose(FLSend);
			
		    int resTemp=sendto(sockfd, buffer,file_size, 0,  
				(struct sockaddr *)&servaddr, sizeof(sockaddr));

			if(buffer!=NULL) delete [] buffer; // adde dy luo;
	
			return resTemp;
		}
		
	

		// ************** Axis calculation class *************

		double AxisCal::RangeCalByGPS(double lati0,double logn0, double lati1, double logn1)
		{
			double X_lati = abs(lati0-lati1)*double(111693.1);
			double X_logn = abs(logn0-logn1)*double(111693.1)*cos(lati0*PI/180.0);

			double range = sqrt(X_lati*X_lati+X_logn*X_logn);

			return range;
		}

		void AxisCal::WGS84ToENV(double lati, double logn, double href, double lati_ref, double logn_ref, double* Pos)
		{
			const double a = double(6378136.49);
			const double b = double(6356755.00);
			const double C45 = 180.0/PI;
			double lat = lati/C45;
			double lon = logn/C45;
			double Px,Py,Pz;
	
			double e2 = (a*a-b*b)/a/a;
			double N = a/sqrt(1-e2*sin(lat)*sin(lat));
			
			// LBH to ECEF 
			Px = (N+href)*cos(lat)*cos(lon);
			Py = (N+href)*cos(lat)*sin(lon);
			Pz = (N*(1-e2)+href)*sin(lat);

			// ECEF to ENV;
			lat = lati_ref/C45; // reference point 
			lon = logn_ref/C45;

			Pos[0] = -Px*sin(lon)+Py*cos(lon);
			Pos[1] = -Px*sin(lat)*cos(lon)-Py*sin(lat)*sin(lon)+Pz*cos(lat);
			Pos[2] =  Px*cos(lat)*cos(lon)+Py*cos(lat)*sin(lon)+Pz*sin(lat);
		}
		
		void AxisCal::ENVToWGS84(double lati, double logn, double* Pos, double* latlogn)
		{// Pos -- Position in ENV 

			// ENV TO ECEF 
			const double C45=180.0/PI;
			double lon=logn/C45;
			double lat=lati/C45;
			double Px,Py,Pz;
			double PE = Pos[0];
			double PN = Pos[1];
			double PU = Pos[2];

			Px = -PE*sin(lon)-PN*sin(lat)*cos(lon)+PU*cos(lat)*cos(lon);
			Py =  PE*cos(lon)-PN*sin(lat)*sin(lon)+PU*cos(lat)*sin(lon);
			Pz =  PN*cos(lat)+PU*sin(lat);

			// ECEF TO WGS84 
			const double a = double(6378136.49);
			const double b = double(6356755.00);
			double e2 = (a*a-b*b)/a/a;
			double logn0 = atan2(Py,Px);
			double lati_ref = lati;

			double N = a/sqrt(1-e2*sin(lati_ref*PI/180.0)*sin(lati_ref*PI/180.0));
			double lati0 = atan((Pz+N*e2*sin(lati_ref*PI/180.0))/sqrt(Px*Px+Py*Py));

			latlogn[0] = lati0*C45;
			latlogn[1] = logn0*C45;

		}

		double AxisCal::CalDirection(double lat_s,double lon_s, double lat_e, double lon_e)
		{
		
			double WD_L = (lat_e - lat_s)*double(111693.1);
			double JD_L = (lon_e - lon_s)*double(111693.1)*cos(lat_e*PI/180.0);
			double angle = atan2(JD_L,WD_L)*180.0/PI;
			if(angle<0) angle += 360.0;

			return angle;

		}


// ********************* Target locating *****************************
   //										by Yunhua-Luo @2016/2/11 

	void Tarlocate::ff(double x[DIM], double f[DIM], rd_par par)
	{// r-d equations ok

		const double a = double(6378136.49);
		const double b = double(6356755.00);

		f[0] = x[0]*x[0]+x[1]*x[1]+x[2]*x[2]-par.rs*par.rs;
		f[1] = par.Vx*x[0]+par.Vy*x[1]+par.Vz*x[2]-par.lambda*par.rs*par.fdc/2;
		f[2] = ((x[0]+par.Px)*(x[0]+par.Px)+(x[1]+par.Py)*(x[1]+par.Py))/(a+par.h_gnd)/(a+par.h_gnd)+
			   (x[2]+par.Pz)*(x[2]+par.Pz)/(b+par.h_gnd)/(b+par.h_gnd)-1; // modified 2016/1/11

	}

	int Tarlocate::gcpelim( int process, double A[DIM][DIM], double xx[DIM] )
	{
			int k,i,j,i0;
			double pelement;
			if( process == 1 ) printf("The process of elimination\n");

			/* elimination step */
			for(k=0;k<DIM;k++)
			{
				/* for principal element*/
			   pelement=fabs(A[k][k]);       i0=k;
			   for(i=k;i<DIM;i++)
			   if( fabs(A[i][k]) > pelement ) { pelement=fabs(A[i][k]); i0=i; }
			   if( i0 != k )
			   {
				  for(j=0;j<DIM;j++)
				  { pelement=A[k][j]; A[k][j]=A[i0][j]; A[i0][j]=pelement; }
				  pelement=xx[k]; xx[k]=xx[i0]; xx[i0]=pelement;
				}
		  if( fabs(A[k][k]) < EPSILON ) return(1);
		  for(i=k+1;i<DIM;i++)
			{
			A[i][k]=A[i][k]/A[k][k];
			for(j=k+1;j<DIM;j++) A[i][j]=A[i][j]-A[i][k]*A[k][j];
			xx[i]=xx[i]-A[i][k]*xx[k];
			}
		  if(process == 1 )
			{
			for(i=0;i<DIM;i++)
			  {
			  for(j=0;j<DIM;j++) printf("%10.6f",A[i][j]);
			  printf("   | %10.6f\n",xx[i]);
			  }
			printf("\n");
			}
		  }

		/* backward step */
		for(i=DIM-1;i>=0;i--)
		   {
			   for(j=i+1;j<DIM;j++) xx[i]=xx[i]-A[i][j]*xx[j];
			   xx[i]=xx[i]/A[i][i];
		   }

		return(0);
	}

	int Tarlocate::secant2( double x[DIM], rd_par par)
	{// 正切、正割法

		int maxiter = MAXITER;
		int i,j,k;
		double norm,h;
		static double x1[DIM],f0[DIM],f1[DIM],A[DIM][DIM];
		h=0.01;
		for(k=0;k<=maxiter;k++)
		  {
		  ff( x, f0, par );
		  for(j=0;j<DIM;j++)
			{
			for(i=0;i<DIM;i++) x1[i]=x[i];
			x1[j]=x[j]+h;
			ff( x1, f1, par );
			norm=0.0;
			for(i=0;i<DIM;i++)  norm=norm+f1[i]*f1[i];
			norm=sqrt(norm);
			if( norm < EPSILON ) goto endd;
			for(i=0;i<DIM;i++) A[i][j]=(f1[i]-f0[i])/h;
			}
		  if( gcpelim( 0, A, f0 ) == 1 )
			{
				printf("The matrix A is singular!\n");
				printf("Strike any key to exit!\n");  
				return -1;
			}
		  norm=0.0;
		  for(i=0;i<DIM;i++)
			{
			norm=norm+f0[i]*f0[i];
			x[i]=x[i]-f0[i];
			}
		  norm=sqrt(norm);
		  if( norm < EPSILON ) goto endd;
		  h=norm;
		  }

		endd:return (k-1);
	}

	void Tarlocate::ENVtoECEF(double lati, double logn, double VE, double VN, double VU, double *Vout)
	{// lati, logn --- angular ok 

		const double C45=180.0/PI;
		double lon=logn/C45;
		double lat=lati/C45;

		Vout[0] = -VE*sin(lon)-VN*sin(lat)*cos(lon)+VU*cos(lat)*cos(lon);
		Vout[1] = VE*cos(lon)-VN*sin(lat)*sin(lon)+VU*cos(lat)*sin(lon);
		Vout[2] = VN*cos(lat)+VU*sin(lat);
	}

	void Tarlocate::WGS84toECEF(double lati, double logn, double href, double *Pos)
	{// WGS84-ECEF ok

		const double a = double(6378136.49);
		const double b = double(6356755.00);
		const double C45 = 180.0/PI;
		double lat = lati/C45;
		double log = logn/C45;
	
		double e2 = (a*a-b*b)/a/a;
		double N = a/sqrt(1-e2*sin(lat)*sin(lat));

		Pos[0] = (N+href)*cos(lat)*cos(log);
		Pos[1] = (N+href)*cos(lat)*sin(log);
		Pos[2] = (N*(1-e2)+href)*sin(lat);

	}

	void Tarlocate::ECEFtoWGS84(double Px, double Py, double Pz, double lati_ref, double *latlogn)
	{// ECEF to WGS84 ok

		const double a = double(6378136.49);
		const double b = double(6356755.00);
		const double C45=180/PI;
		double e2 = (a*a-b*b)/a/a;
		double logn0 = atan2(Py,Px);

		double N = a/sqrt(1-e2*sin(lati_ref*PI/180.0)*sin(lati_ref*PI/180.0));
		double lati0 = atan((Pz+N*e2*sin(lati_ref*PI/180.0))/sqrt(Px*Px+Py*Py));

		latlogn[0] = lati0*C45;
		latlogn[1] = logn0*C45;
	}
	
	double Tarlocate::CalDirection(double lat_s,double lon_s, double lat_e, double lon_e)
	{
		double WD_L = (lat_e - lat_s)*double(111693.1);
		double JD_L = (lon_e - lon_s)*double(111693.1)*cos(lat_e*PI/180.0);

		double angle = atan2(JD_L,WD_L)*180.0/PI;
		if(angle < 0) angle += 360.0;

		return angle;
	}

	void Tarlocate::ArbitaryPosCal(double *pos_info, long *grid_info, long px, long py, double *latlogn)
	{// Arbitary position calculation dev by 2015/12/19 ok

		long a_1 = grid_info[0];
		long b_1 = grid_info[1];

		long a_2 = grid_info[2];
		long b_2 = grid_info[3];

		long a_3 = grid_info[4];
		long b_3 = grid_info[5];

		long a_4 = px;
		long b_4 = py;

		double lon_1 = pos_info[0];
		double lat_1 = pos_info[1];

		double lon_2 = pos_info[2];
		double lat_2 = pos_info[3];

		double lon_3 = pos_info[4];
		double lat_3 = pos_info[5];

		// 二元矩阵的逆的系数设为A
		double A=(a_3-a_1)*(b_2-b_1)-(b_3-b_1)*(a_2-a_1);

		// 在a向上纬度差的系数乘以A后设为La_a,经度差的系数乘以A后设为Lo_a;
		double La_a = (b_2-b_1)*(lat_3-lat_1)-(b_3-b_1)*(lat_2-lat_1);
		double Lo_a = (b_2-b_1)*(lon_3-lon_1)-(b_3-b_1)*(lon_2-lon_1);

		// 在b向上纬度差的系数乘以A后设为La_b,经度差的系数乘以A后设为Lo_b;
		double La_b = (a_3-a_1)*(lat_2-lat_1)-(a_2-a_1)*(lat_3-lat_1);
		double Lo_b = (a_3-a_1)*(lon_2-lon_1)-(a_2-a_1)*(lon_3-lon_1);

		// 已知(a_4,b_4),以1点为参考，其经纬度计算公式为：
		latlogn[1] = lon_1+Lo_a/A*(a_4-a_1)+Lo_b/A*(b_4-b_1);
		latlogn[0] = lat_1+La_a/A*(a_4-a_1)+La_b/A*(b_4-b_1);

	}

    void Tarlocate::Geo_locating(rd_par_input par_in, double *latlogn)
	{// locating by geometry , resolution < 1km

		double ve = par_in.ve;
		double vn = par_in.vn;
		double rs = par_in.r0;
		double href = par_in.href;
		double lati0 = par_in.lati;
		double logn0 = par_in.logn;
		double lookside = par_in.lookside;

		double yaw_res = 0.3*PI/180.0;	// electronic scan residual angle 
		double fly_ang= atan2(ve,vn);
		
		double Rg = sqrt(rs*rs-href*href);	
		double X_JD_t = -Rg*lookside*cos((fly_ang-yaw_res)*lookside);        // fly_ang-yaw_ang == true_fly_ang
		double Y_WD_t = Rg*sin((fly_ang-yaw_res)*lookside);					 // notice tar_azimuth  

		latlogn[0] = lati0 + double(Y_WD_t)/double(111693.1);
		latlogn[1] = logn0 + double(X_JD_t)/double(111693.1)/cos(lati0*PI/180.0); 

	}

	void Tarlocate::RD_locating(rd_par_input par_in, double *latlogn_rd, double *latlogn_geo)
	{// R-D locating modual dev by Yunhua-Luo 2015/12/19

		// step 0. resolve initial value by geo-locating 
		double Pos_init[3];
		Geo_locating(par_in,latlogn_geo);
		WGS84toECEF(latlogn_geo[0], latlogn_geo[1], par_in.h_gnd,Pos_init);

		// step 1. NEU velocity to ECEF velocity
		double Vout[3],Pos[3];
		ENVtoECEF(par_in.lati, par_in.logn, par_in.ve, par_in.vn, par_in.vu, Vout);
	
		// step 2. airplane WGS84 -> ECEF 
		WGS84toECEF(par_in.lati, par_in.logn, par_in.href,Pos);

		// step 3. parameters 
		static double x0[DIM] = {Pos_init[0]-Pos[0],Pos_init[1]-Pos[1],Pos_init[2]-Pos[2]};
		//static double x0[DIM] = {0,0,1};
	
		rd_par par;
		par.fdc = par_in.fdc;
		par.h_gnd = par_in.h_gnd;
		par.lambda = par_in.lambda;
		par.rs = par_in.r0-150.0;

		par.Px = Pos[0];
		par.Py = Pos[1];
		par.Pz = Pos[2];
		par.Vx = Vout[0];
		par.Vy = Vout[1];
		par.Vz = Vout[2];

		// step 3. resolve r-d locating nonlinear-equations
		double Px,Py,Pz;
		int iter = secant2(x0,par);
		if(iter==-1 || iter >= MAXITER) // non-convanice 
		{
			memcpy(latlogn_rd,latlogn_geo,sizeof(double)*2);
		}
		else
		{
			Px = x0[0] + par.Px;
			Py = x0[1] + par.Py;
			Pz = x0[2] + par.Pz;

			// step 4. Transform ECEF to WGS84 
			ECEFtoWGS84(Px, Py, Pz,latlogn_geo[0],latlogn_rd);
			//ECEFtoWGS84(Px, Py, Pz,par_in.lati,latlogn_rd);  // 
		}

		// ********** lookside verify ********** Added at 2016/02/19
		double lati_cal = latlogn_rd[0];
		double logn_cal = latlogn_rd[1];
		long AziAngle = long(CalDirection(par_in.lati,par_in.logn,lati_cal,logn_cal)); 
		long FlyAngle = long(atan2(par_in.ve,par_in.vn)*180.0/PI);
		if(FlyAngle < 0) FlyAngle += 360.0;

		float look_side_real;
		if((AziAngle-FlyAngle+360)%360 >= 180) look_side_real = 1;   // left  side;
		else								   look_side_real = -1;  // right side;

		if(look_side_real != par_in.lookside)		// Mirror
		{
			latlogn_rd[0] = par_in.lati*2-lati_cal;
			latlogn_rd[1] = par_in.logn*2-logn_cal;
		}

		// add constraints
		if(abs(latlogn_rd[0])>90 || abs(latlogn_rd[1]) > 180)
		{
			memcpy(latlogn_rd,latlogn_geo,sizeof(double)*2);
		}
}




// **************** Geotiff Read and Write functions **********************
//                                    dev by Yunhua-Luo @2016/7/11 

int GeoTiff::GeotiffWriteFromFile(const char *infname,const char *outfname,double *adfGeoTransform,char *WKT)
{
	GDALAllRegister();
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8","NO"); 
	GDALDriver *poDriver = GetGDALDriverManager()->GetDriverByName("GTIFF");
	if( poDriver == NULL )
	{
		return FALSE;
	}

	setlocale(LC_ALL, "chs");

	GDALDataset* inData=(GDALDataset*)GDALOpen(infname,GA_ReadOnly);
	if(inData)
	{
		int xSize = inData->GetRasterXSize();//图象宽度
		int ySize = inData->GetRasterYSize();//图象高度
		int nBand = inData->GetRasterCount();//波段数量
		GDALDataType iDataType = inData->GetRasterBand(1)->GetRasterDataType();
		//GDAL读取存储
		if ( iDataType == GDT_Byte ){
			BYTE * pBuffer = new BYTE[xSize*ySize];
			GDALRasterBand *pBand;
			pBand = inData->GetRasterBand(1);
			pBand->RasterIO(GF_Read, 0, 0, xSize, ySize, pBuffer, xSize, ySize,iDataType, 0,0);

			GDALDataset *outData = poDriver->Create( outfname, xSize, ySize, nBand, iDataType, NULL );
			if(outData!=NULL)
			{
				outData->SetGeoTransform(adfGeoTransform);
				outData->SetProjection(WKT);
				outData->GetRasterBand(1)->RasterIO(GF_Write, 0, 0, xSize,ySize,pBuffer, xSize, ySize,iDataType,0,0);
				GDALClose(outData);
			}
			delete []pBuffer;
		}
		else if (iDataType == GDT_UInt16){
			unsigned short * pBuffer = new unsigned short [xSize*ySize];
			GDALRasterBand *pBand;
			pBand = inData->GetRasterBand(1);
			pBand->RasterIO(GF_Read, 0, 0, xSize, ySize, pBuffer, xSize, ySize,iDataType, 0,0);

			GDALDataset *outData = poDriver->Create( outfname, xSize, ySize, nBand, iDataType, NULL );
			if(outData!=NULL)
			{
				outData->SetGeoTransform(adfGeoTransform);
				outData->SetProjection(WKT);
				outData->GetRasterBand(1)->RasterIO(GF_Write, 0, 0, xSize,ySize,pBuffer, xSize, ySize,iDataType,0,0);
				GDALClose(outData);
			}
			delete []pBuffer;
		}
		else if (iDataType == GDT_Int16){
			short * pBuffer = new short [xSize*ySize];
			GDALRasterBand *pBand;
			pBand = inData->GetRasterBand(1);
			pBand->RasterIO(GF_Read, 0, 0, xSize, ySize, pBuffer, xSize, ySize,iDataType, 0,0);

			GDALDataset *outData = poDriver->Create( outfname, xSize, ySize, nBand, iDataType, NULL );
			if(outData!=NULL)
			{
				outData->SetGeoTransform(adfGeoTransform);
				outData->SetProjection(WKT);
				outData->GetRasterBand(1)->RasterIO(GF_Write, 0, 0, xSize,ySize,pBuffer, xSize, ySize,iDataType,0,0);
				GDALClose(outData);
			}
			delete []pBuffer;
		}		
	}
	GDALClose((GDALDatasetH)inData);
	return 1;
}

int GeoTiff::Tiff2Geotiff(double *point_lati, double *point_logn, long imgW, long imgH, const char *InImagefile,const char *OutImagfile)
{
	// 点顺序： 左上、右上、左下、右下
	
	// 改为： 左上、左下，右上，右下；
	//计算图像四角图像坐标
	GDAL_GCP pGCPs[4];
	
	pGCPs[0].dfGCPPixel = 0;							pGCPs[0].dfGCPLine = 0;
	pGCPs[1].dfGCPPixel = imgW;							pGCPs[1].dfGCPLine = 0;
	pGCPs[2].dfGCPPixel = 0;							pGCPs[2].dfGCPLine = imgH;
	pGCPs[3].dfGCPPixel = imgW;							pGCPs[3].dfGCPLine = imgH;

	for (int i =0;i <4; i++)
	{
		pGCPs[i].dfGCPX=point_logn[i];
		pGCPs[i].dfGCPY=point_lati[i];
	}

	double adfGeoTransform[6]={0,0,0,0,0,0};
	GDALGCPsToGeoTransform( 4, pGCPs, adfGeoTransform, TRUE ); //gdal库的函数
	//设置坐标系统投影
	char* gcs_wgs1984="GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.01745329251994328,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]]";
	
	setlocale(LC_ALL, "chs");
	GeotiffWriteFromFile(InImagefile,OutImagfile,adfGeoTransform,gcs_wgs1984); //调用下面的写geotiff函数

	return 1;
}

int GeoTiff::GeotiffWriteFromByteData(BYTE *pBuffer,long imgH, long imgW, const char *outfname,double *adfGeoTransform,char *WKT)
{

	GDALAllRegister();
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8","NO"); 
	GDALDriver *poDriver = GetGDALDriverManager()->GetDriverByName("GTIFF");
	if( poDriver == NULL )
	{
		return FALSE;
	}
	
	int xSize = imgW;//图象宽度
	int ySize = imgH;//图象高度
	int nBand = 1;	//波段数量
	
	setlocale(LC_ALL, "chs");
	GDALDataset *outData = poDriver->Create(outfname, xSize, ySize, nBand, GDT_Byte, NULL );
	if(outData!=NULL)
	{
		outData->SetGeoTransform(adfGeoTransform);
		outData->SetProjection(WKT);
		outData->GetRasterBand(1)->RasterIO(GF_Write, 0, 0, xSize,ySize,pBuffer, xSize, ySize,GDT_Byte,0,0);
		GDALClose(outData);
	}

	return 1;

}

int GeoTiff::WriteByte2Geotiff(double *point_lati, double *point_logn, long imgW, long imgH, BYTE *data,const char *OutImagfile)
{
	// 点顺序： 左上、右上、左下、右下

	//计算图像四角图像坐标
	
	GDAL_GCP pGCPs[4];
	pGCPs[0].dfGCPPixel = 0;							pGCPs[0].dfGCPLine = 0;
	pGCPs[1].dfGCPPixel = imgW;							pGCPs[1].dfGCPLine = 0;
	pGCPs[2].dfGCPPixel = 0;							pGCPs[2].dfGCPLine = imgH;
	pGCPs[3].dfGCPPixel = imgW;							pGCPs[3].dfGCPLine = imgH;

	for (int i=0;i<4; i++)
	{
		pGCPs[i].dfGCPX=point_logn[i];
		pGCPs[i].dfGCPY=point_lati[i];
	}

	double adfGeoTransform[6]={0,0,0,0,0,0};
	GDALGCPsToGeoTransform( 4, pGCPs, adfGeoTransform, TRUE ); //gdal库的函数
	//设置坐标系统投影
	char* gcs_wgs1984="GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.01745329251994328,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]]";
	
	setlocale(LC_ALL, "chs");
	GeotiffWriteFromByteData(data,imgH, imgW, OutImagfile,adfGeoTransform,gcs_wgs1984); //调用下面的写geotiff函数

	return 1;
}

int GeoTiff::GeotiffWriteFrom16BitData(unsigned short int *pBuffer,long imgH, long imgW, const char *outfname,double *adfGeoTransform,char *WKT)
{

	GDALAllRegister();
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8","NO"); 
	GDALDriver *poDriver = GetGDALDriverManager()->GetDriverByName("GTIFF");
	if( poDriver == NULL )
	{
		return FALSE;
	}
	
	int xSize = imgW;//图象高度
	int ySize = imgH;//图象宽度
	int nBand = 1;//波段数量
	
	setlocale(LC_ALL, "chs");
	GDALDataset *outData = poDriver->Create(outfname, xSize, ySize, nBand, GDT_UInt16, NULL );
	if(outData!=NULL)
	{
		outData->SetGeoTransform(adfGeoTransform);
		outData->SetProjection(WKT);
		outData->GetRasterBand(1)->RasterIO(GF_Write, 0, 0, xSize,ySize,pBuffer, xSize, ySize,GDT_UInt16,0,0);
		GDALClose(outData);
	}

	return 1;

}

int GeoTiff::Write16Bit2Geotiff(double *point_lati, double *point_logn, long imgW, long imgH, unsigned short int *data,const char *OutImagfile)
{
	// 点顺序： 左上、右上、左下、右下

	//计算图像四角图像坐标
	GDAL_GCP pGCPs[4];
	pGCPs[0].dfGCPPixel = 0;							pGCPs[0].dfGCPLine = 0;
	pGCPs[1].dfGCPPixel = imgW;							pGCPs[1].dfGCPLine = 0;
	pGCPs[2].dfGCPPixel = 0;							pGCPs[2].dfGCPLine = imgH;
	pGCPs[3].dfGCPPixel = imgW;							pGCPs[3].dfGCPLine = imgH;

	for (int i =0;i <4; i++)
	{
		pGCPs[i].dfGCPX=point_logn[i];
		pGCPs[i].dfGCPY=point_lati[i];
	}
	double adfGeoTransform[6]={0,0,0,0,0,0};
	GDALGCPsToGeoTransform( 4, pGCPs, adfGeoTransform, TRUE ); //gdal库的函数
	//设置坐标系统投影
	char* gcs_wgs1984="GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.01745329251994328,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]]";
	
	setlocale(LC_ALL, "chs");
	GeotiffWriteFrom16BitData(data,imgH, imgW, OutImagfile,adfGeoTransform,gcs_wgs1984); //调用下面的写geotiff函数

	return 1;
}

int GeoTiff::Read2Geotiff(const char *infname, BYTE *pBuffer, long *info, double *adfGeoTransform)
{
	GDALAllRegister();
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8","NO"); 
	GDALDriver *poDriver = GetGDALDriverManager()->GetDriverByName("GTIFF");
	if( poDriver == NULL )
	{
		return FALSE;
	}

	setlocale(LC_ALL, "chs");
	GDALDataset* inData=(GDALDataset*)GDALOpen(infname,GA_ReadOnly);
	if(inData)
	{
		int xSize = inData->GetRasterXSize();//图象宽度
		int ySize = inData->GetRasterYSize();//图象高度
		int nBand = inData->GetRasterCount();//波段数量
		GDALDataType iDataType = inData->GetRasterBand(1)->GetRasterDataType();
		
		info[0] = xSize;
		info[1] = ySize;

		//GDAL读取存储
		if(iDataType == GDT_Byte)
		{
			GDALRasterBand *pBand;
			pBand = inData->GetRasterBand(1);
			inData->GetGeoTransform(adfGeoTransform);
			pBand->RasterIO(GF_Read, 0, 0, xSize, ySize, pBuffer, xSize, ySize,iDataType, 0,0);
			
		}
		GDALClose(inData);

		return TRUE;

	}
		
	return FALSE;
}

