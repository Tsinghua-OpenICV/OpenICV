//  @ Project : RADAR
//  @ File Name : sensorlrr.h
//  @ Date : 2016/12/22
//  @ Author :
//
//

#ifndef _XiaopengAPS_H
#define _XiaopengAPS_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
#include "OpenICV/Basis/icvStructureData.hxx"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/structure/structureCanTuan.h"
#include "OpenICV/structure/structureVehinfo.h"
#include "OpenICV/structure/structure_gps.h"
#include "OpenICV/structure/structureIMU.h"
#include "OpenICV/Net/icvUdpReceiverSource.h"

#include <cstdlib>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>

#include  "CanEPS_511.hpp" 
#include  "CanSPD_510.hpp" 
#include  "CanBT_509.hpp" 
#include  "CanMotor_50B.hpp" 
#include  "CanCOM_5F0.hpp"  

#include "CanFrame.h"
#include <iomanip>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <bitset>
#include <unistd.h>
#include "parkingfunc.hpp"

#define PI 3.1415926

typedef unsigned char uint8;
typedef float float32;
typedef double float64;
typedef char int8;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned int uint32;
typedef int int32;

using namespace icv;
using namespace std;
using namespace core;
using namespace icv::function;
typedef data::icvStructureData<veh_info> icvveh_info;
typedef data::icvStructureData<Imu> icvImu;
typedef data::icvStructureData<donnees_gps> icvGPS;
 typedef data::icvStructureData<NavSatFix>    icvNavSatFix;
  typedef data::icvStructureData<TwistWithCovarianceStamped>    icvTwistWithCovarianceStamped;
  typedef data::icvStructureData<Odometry>    icvOdometry;


class Xiaopengaps : public icvFunction
{
  public:
	Xiaopengaps(icv_shared_ptr<const icvMetaData> info) : icvFunction(info)
	{
		load_curvemap(); //导入地图
		outvalue=new data::icvFloat32Data();

	};


	void load_curvemap()
	{
		bestLmap.open(bestLmap_ads);
		BezierKmap.open(BezierKmap_ads); //
		int indexf = 0;
		int mapi = 0;
		int mapj = 0;
		int mapk = 0;
		std::cout << "Loading curve map" << endl;
		while (bestLmap.good() && indexf < 290000)
		{
			mapi = floor(indexf / 2900);
			mapj = floor((indexf - mapi * 2900) / 29);
			mapk = indexf - mapi * 2900 - mapj * 29;
			bestLmap >> bestL[mapi][mapj][mapk];
			indexf = indexf + 1;
		}
		indexf = 0;
		mapi = 0;
		mapj = 0;
		mapk = 0;
		while (BezierKmap.good() && indexf < 290000)
		{
			mapi = floor(indexf / 2900);
			mapj = floor((indexf - mapi * 2900) / 29);
			mapk = indexf - mapi * 2900 - mapj * 29;
			BezierKmap >> BezierK[mapi][mapj][mapk];
			indexf = indexf + 1;
		}
		for (mapi = 0; mapi < 10; mapi++)
		{
			wheelout[mapi] = 0;
		}
		std::cout << "Loading  map finish" << endl;
		std::cout << "Try final bestL[%f]" << bestL[0][0][0] << endl;
		std::cout << "Try final BezierK[%f]" << BezierK[0][0][0] << endl;
		std::cout << "Try final bestL[%f]" << bestL[99][99][28] << endl;
		std::cout << "Try final BezierK[%f]" << BezierK[99][99][28] << endl;
		control = 1;
		car_Status.status = 1;
		load_aps_map = true;
	}
	void get_tragectory()
	{

		std::cout << "startPoint.x= " << startPoint.x << std::endl;
		if (startPoint.x >= WW / 2) //判断方位
		{
			std::cout << "RIGHT POSITION " << std::endl;
			OptArconlineright(&ArcKMap[0][0][0]);
			traceGeneration2(&ControlPoint2[0], startPoint.x, startPoint.y, startPoint.angle * pi_aps / 180);
		}
		else
		{
			std::cout << "LEFTPOSITION " << std::endl;
			OptArconlineleft(&ArcKMap[0][0][0]);
			traceGeneration2left(&ControlPoint2[0], startPoint.x, startPoint.y, startPoint.angle * pi_aps / 180);
		}

		for (int fgdfg = 0; fgdfg < 8; fgdfg++)
		{
			//std::cout << "fgdfg=" << fgdfg << std::endl;
			std::cout << "ControlPoint2=" << ControlPoint2[fgdfg] << std::endl;
		}
		if (ControlPoint2[0] == 0)
		{
			std::cout << "WARING!!!! WRONG POSTION!!! CANT GENERATE TRAJECTORY" << std::endl;
		}
		else
		{
			if (startPoint.x >= WW / 2) //判断方位
			{
				generateCurveList2(control, ControlPoint2, startPoint, aps_points);
			}
			else
			{
				generateCurveList2left(control, ControlPoint2, startPoint, aps_points);
			}
			int i;
			for (i = 1; i < aps_points.size(); i++)
			{
				aps_points[i].Road_angle = atan2((aps_points[i - 1].y - aps_points[i].y), (aps_points[i - 1].x - aps_points[i].x)) * 180 / pi_aps;
			}
		}
	}
  virtual void Execute(std::vector<icvNodeInput*>& inputPorts, std::vector<icvNodeOutput*>& outputPorts) override
	{
		
	imu_data=read_Input<icvImu>(0);
	gps_data=read_Input<icvNavSatFix>(1);	
		
if(is_not_empty(0)&&is_not_empty(1))
{
ICV_LOG_INFO<<"IMU DATA:"<<imu_data.linear_acceleration.z;
		//转换坐标
		lon_alt2xyzCallback();//to do 没有坐标会怎样

		//获取第一个点
		while (havestart == false) //&&x_vehicle_gps ！ = 0) // 如何没有初始定位就一直读到有第一个定位 //to do 怎么确定是第一个
		{
			startPoint.x = trans_p.x; //x
			startPoint.y = trans_p.y; //y
			startPoint.status = 0;
			startPoint.angle = trans_p.angle; //heading
			havestart == true;
			sleep(0.5);
			cout << "waiting for start_position" << endl;
		}
		while (have_trag == false) //有定位后开始规划
		{
			get_tragectory();
			have_trag==true;
		}
		/******************************
		* 控制
  		*****************************/
		int count_now = 0;

		strangle = ControlStr2(aps_points, trans_p.x, trans_p.y, trans_p.angle, count_now, ACC_v); //阿克满转交konga
		ICV_LOG_INFO<<"out put steering angle: "<<strangle;
		outvalue->setoutvalue(strangle);
		Send_Out(outvalue,0);

}
		//输出strangle
	}
	// Dengnanshan 6-18 input raw gpsimu  output transfor position ，heading
	void lon_alt2xyzCallback()
	{

		double scale[2];
		float lat, lon;
		lat = gps_data.latitude;
		lon = gps_data.longitude;
		const float lat1 = 40.0068269; //雷涛2019.3.27汽研所左上角羽毛球左下角
		const float lon1 = 116.3281442;
		float parkangle = PI; //停车位角度

		float angle_imu;
		//lon[1]=116.330256125;  //meishuguan,lianggewan(yizhiyong)
		//lat[1]=39.9991777542;
		//lon[1]=116.330689964;  //meishuguantoqiyanshuo11111111
		//lat[1]=40.006710;//邓楠山2018年10月30日气研所中心
		//lon[1]=116.328079;
		//lat[1]=40.0065521;//邓楠山2018年11月14日1日气焰所第二个车位
		//lon[1]=116.328137;40.0068269
		//lat[1]=40.006787;//雷涛2018.12.10气焰所羽毛球左下角
		//lon[1]=116.3280836;
		//lat[1]=40.0068269;//雷涛2019.3.27汽研所左上角羽毛球左下角
		//lon[1]=116.3281442;
		//parkangle=PI;
		//lat[1]=40.006766;//汽研所2019.1.15中央点
		//lon[1]=116.3280142;
		// lat[1]=39.9996645;//邓楠山2018年11月14日1日美术馆停车场
		// lon[1]=116.3303432;
		a = 6378137.0;
		f = 298.2572;
		b = (f - 1) / f * a;
		e2 = (a * a - b * b) / (a * a);
		A = a * (1 - e2) / pow((1 - e2 * pow(sin(lat1 / 180.0 * PI), 2)), 1.5);
		B = a * cos(lat1 / 180.0 * PI) / sqrt(1 - e2 * pow(sin(lat1 / 180.0 * PI), 2));
		scale[1] = B * 1.0 / 180.0 * PI;
		scale[2] = A * 1.0 / 180.0 * PI;
		xx_t = (lon - lon1) * scale[1]; //转换为坐标
		yy_t = (lat - lat1) * scale[2];

		//3-29 转换为泊车坐标
		xx_aps = cos(parkangle) * xx_t + sin(parkangle) * yy_t;
		yy_aps = -sin(parkangle) * xx_t + cos(parkangle) * yy_t;

		//IMU
		float x = imu_data.orientation.x;
		float y = imu_data.orientation.y;
		float z = imu_data.orientation.z;
		float w = imu_data.orientation.w;
		float course_angle_rad = atan2f(2 * (w * z + x * y), w * w + x * x - y * y - z * z) * 180 / 3.1415926;

		//规到制定范围
		if ((course_angle_rad >= 90) && (course_angle_rad < 180)) //三象限
		{
			angle_imu = -270 + course_angle_rad;
		}

		else
		{
			{
				angle_imu = 90 + course_angle_rad;
			}
		}
		angle_imu = angle_imu + parkangle * 180 / PI;
		if (angle_imu > 180)
		{
			angle_imu = angle_imu - 360;
		}
		else if (angle_imu < -180)
		{
			angle_imu = angle_imu + 360;
		}
		else
		{
			angle_imu = angle_imu;
		}
		trans_p.x = xx_aps;
		trans_p.y = yy_aps;
		trans_p.angle = angle_imu;

		/*
	 	output  xx_aps yy_aps  angle_imu
	 */
	}
void traceGeneration2(float *ControlPoint2, float startx, float starty, float startangle) 
{
  float xc, yc, anglec;
  float xAmax, yAmax, angleAmax, xgrid, ygrid, anglegrid;
  yc = starty;
  anglec = startangle;
  anglec = anglec + pi_aps;
  xc=startx;
  yc=starty;
  xAmax = 20.0;
  yAmax = 20.0;
  angleAmax = pi_aps;
  xgrid = xAmax / 100.0;
  ygrid = yAmax / 100.0;
  anglegrid = angleAmax / 30.0;
  float NewBmap[100][100][29];
  float NewbestL[100][100][29];
  float finalmap[100][100][29];
  float x;
  float y;
  float angle;
  int mi, mj, mk;
  float anglenow, xnow, ynow;
  for (mi = 0; mi < 100; mi++)
  {
    for (mj = 0; mj < 100; mj++)
    {
      for (mk = 0; mk < 29; mk++)
      {
        x = xgrid * (mi+1) + WW / 2;
        y = ygrid * (mj+1) + LLL;
        angle = anglegrid * (mk+1) - pi_aps / 2;
        angle=angle+pi_aps;
        anglenow = (angle - anglec);
        xnow = cos(anglec) * (x - xc) + sin(anglec) * (y - yc);
        ynow = -sin(anglec) * (x - xc) + cos(anglec) * (y - yc);
        if (anglenow < -pi_aps / 2 || anglenow > pi_aps / 2 || xnow > 20 || xnow < 0 || ynow > 10 || ynow < -10)
        {
          NewBmap[mi][mj][mk] = 1 / R_min;
          NewbestL[mi][mj][mk] = 0.3;
        }
        else
        {

          int intx = floor((xnow) / 20 * 100);
          int inty = floor((ynow) / 20 * 100) + 50;
          int intangle = floor(anglenow / pi_aps * 30) + 15;
          if (intx < 0 || intx > 98 || inty < 0 || inty > 98 || intangle < 0 || intangle > 27)
          {
            NewBmap[mi][mj][mk] = 1 / R_min;
            NewbestL[mi][mj][mk] = 0.3;
          }
          else
          {
            NewBmap[mi][mj][mk] = (BezierK[intx][inty][intangle] + BezierK[intx + 1][inty][intangle] + BezierK[intx][inty + 1][intangle] + BezierK[intx + 1][inty + 1][intangle]) / 4;
            NewbestL[mi][mj][mk] = (bestL[intx][inty][intangle] + bestL[intx + 1][inty][intangle] + bestL[intx][inty + 1][intangle] + bestL[intx + 1][inty + 1][intangle]) / 4;
          }
        }
      }
    }
  }
  for (mi = 0; mi < 100; mi++)
  {
    for (mj = 0; mj < 100; mj++)
    {
      for (mk = 0; mk < 29; mk++)
      {
        if (NewBmap[mi][mj][mk] < ArcKMap[mi][mj][mk])
        {
         
          finalmap[mi][mj][mk] = ArcKMap[mi][mj][mk]; 
        }
        else
        {
         finalmap[mi][mj][mk] = NewBmap[mi][mj][mk];
        }
      }
    }
  }
  float KK = 100;
  int xi, yi, ti;
  for (mi = 0; mi < 100; mi++)
  {
    for (mj = 0; mj < 100; mj++)
    {
      for (mk = 10; mk < 28; mk++)
      {
        if (finalmap[mi][mj][mk] < KK && ArcKMap[mi][mj][mk] <( 1 / R_min))
        {
          KK = finalmap[mi][mj][mk];
          xi = mi;
          yi = mj;
          ti = mk;
          std::cout << "KK=" << KK << "xi,yi,ti=" << xi << " " << yi << " " << ti << std::endl;
        }
      }
    }
  }
 if (KK<=1/R_min)
 {
  float xresult, yresult, tresult;
  xresult = float(xi+1) * xgrid + WW / 2;
  yresult = float(yi+1) * ygrid + LLL;
  tresult = float(ti+1) * anglegrid - pi_aps / 2;
  float S, Rtempp;
  Rtempp = (xresult - WW / 2) / (1 - sin(tresult));
  S = yresult - Rtempp * cos(tresult);
  *ControlPoint2 = xresult;
  *(ControlPoint2 + 1) = yresult;
  *(ControlPoint2 + 2) = WW / 2;
  *(ControlPoint2 + 3) = S;
  *(ControlPoint2 + 4) = WW / 2;
  *(ControlPoint2 + 5) = L_r + delta_1;
  *(ControlPoint2 + 6) = tresult;
  if (NewbestL[(int)xi][(int)yi][(int)ti]<0.3)
  {
    NewbestL[(int)xi][(int)yi][(int)ti]=0.4;
  }
  *(ControlPoint2 + 7) = NewbestL[(int)xi][(int)yi][(int)ti];
 }
 else
 {
   int waringaaa=100;
   while(waringaaa>0)
   {
     waringaaa=waringaaa-1;
    //ROS_INFO("WARING!!!! WRONG POSTION!!! CANT GENERATE TRAJECTORY");
    
   }
   *ControlPoint2 = 0;
 }
}
void traceGeneration2left(float *ControlPoint2, float startx, float starty, float startangle) 
{
  float xc, yc, anglec;
  float xAmax, yAmax, angleAmax, xgrid, ygrid, anglegrid;
  yc = starty;
  anglec = startangle;
  anglec = anglec + pi_aps;
  xc=startx;
  yc=starty;
  xAmax = 20.0;
  yAmax = 20.0;
  angleAmax = pi_aps;
  xgrid = xAmax / 100.0;
  ygrid = yAmax / 100.0;
  anglegrid = angleAmax / 30.0;
  float NewBmap[100][100][29];
  float NewbestL[100][100][29];
  float finalmap[100][100][29];
  float x;
  float y;
  float angle;
  int mi, mj, mk;
  float anglenow, xnow, ynow;
  for (mi = 0; mi < 100; mi++)//筛选是否在区间内
  {
    for (mj = 0; mj < 100; mj++)
    {
      for (mk = 0; mk < 29; mk++)
      {
        x =- xgrid * (mi+1) + WW / 2;
        y = ygrid * (mj+1) + LLL;
        angle = anglegrid * (mk+1) + pi_aps / 2;
        angle=angle+pi_aps;
        //这里出现越界
        anglenow = (angle - anglec); //anglec不大于180，会变成-180
        
        xnow = cos(anglec) * (x - xc) + sin(anglec) * (y - yc);
        ynow = -sin(anglec) * (x - xc) + cos(anglec) * (y - yc);

        //防止越界
        if(anglenow>pi_aps)
        {
          anglenow=anglenow-2*pi_aps;
        }
        else if(anglenow<-pi_aps)
        {
          anglenow=anglenow+2*pi_aps;
        }
        else{}

        if (anglenow < -pi_aps / 2 || anglenow > pi_aps / 2 || xnow > 20 || xnow < 0 || ynow > 10 || ynow < -10)
        {
          NewBmap[mi][mj][mk] = 1 / R_min;
          NewbestL[mi][mj][mk] = 0.3;
        }
        else
        {          
          int intx = floor((xnow) / 20 * 100);
          int inty = floor((ynow) / 20 * 100) + 50;
          int intangle = floor(anglenow / pi_aps * 30) + 15;
          if (intx < 0 || intx > 98 || inty < 0 || inty > 98 || intangle < 0 || intangle > 27)
          {
            NewBmap[mi][mj][mk] = 1 / R_min;
            NewbestL[mi][mj][mk] = 0.3;
          }
          else
          {
            NewBmap[mi][mj][mk] = (BezierK[intx][inty][intangle] + BezierK[intx + 1][inty][intangle] + BezierK[intx][inty + 1][intangle] + BezierK[intx + 1][inty + 1][intangle]) / 4;
            NewbestL[mi][mj][mk] = (bestL[intx][inty][intangle] + bestL[intx + 1][inty][intangle] + bestL[intx][inty + 1][intangle] + bestL[intx + 1][inty + 1][intangle]) / 4;
          }
        }
      }
    }
  }
  for (mi = 0; mi < 100; mi++)//确定最终map
  {
    for (mj = 0; mj < 100; mj++)
    {
      for (mk = 0; mk < 29; mk++)
      {
        
        if (NewBmap[mi][mj][mk] < ArcKMap[mi][mj][mk])
        {
         
          finalmap[mi][mj][mk] = ArcKMap[mi][mj][mk]; 
        }
        else
        {
         finalmap[mi][mj][mk] = NewBmap[mi][mj][mk];
        }
        //ROS_INFO("finalmap POINT[%f]",finalmap[mi][mj][mk]);
      }
    }
  }
  float KK = 100;
  int xi, yi, ti;
  for (mi = 0; mi < 100; mi++)//筛选结果
  {
  //    ROS_INFO("mi= [%d]",mi);
    for (mj = 0; mj < 100; mj++)
    {
      for (mk = 0; mk < 28; mk++)
      {
      //  ROS_INFO("ArcKMap[%d][%d][%d]= [%f]",mi,mj,mk,ArcKMap[mi][mj][mk]);
        if (finalmap[mi][mj][mk] < KK && ArcKMap[mi][mj][mk] <( 1 / R_min))
        {
          KK = finalmap[mi][mj][mk];
          xi = mi;
          yi = mj;
          ti = mk;
          std::cout << "KK=" << KK << "xi,yi,ti=" << xi << " " << yi << " " << ti << std::endl;
        }
      }
    }
  }
 if (KK<=1/R_min)//求解结果
 {
  float xresult, yresult, tresult;
  xresult = -float(xi+1) * xgrid + WW / 2;
  yresult = float(yi+1) * ygrid + LLL;
  tresult = float(ti+1) * anglegrid + pi_aps / 2;
  float S, Rtempp;
  Rtempp = fabs(xresult - WW / 2) / (1 - sin(tresult));
  S = yresult + Rtempp * cos(tresult);
  *ControlPoint2 = xresult;
  *(ControlPoint2 + 1) = yresult;
  *(ControlPoint2 + 2) = WW / 2;
  *(ControlPoint2 + 3) = S;
  *(ControlPoint2 + 4) = WW / 2;
  *(ControlPoint2 + 5) = L_r + delta_1;
  *(ControlPoint2 + 6) = tresult;
  if (NewbestL[(int)xi][(int)yi][(int)ti]<0.3)
  {
    NewbestL[(int)xi][(int)yi][(int)ti]=0.4;
  }
  *(ControlPoint2 + 7) = NewbestL[(int)xi][(int)yi][(int)ti];
 }
 else
 {
   int waringaaa=100;
   while(waringaaa>0)
   {
     waringaaa=waringaaa-1;
   // ROS_INFO("WARING!!!! WRONG POSTION!!! CANT GENERATE TRAJECTORY");
    
   }
   *ControlPoint2 = 0;
 }
}
  private:
	static const int MaxUdpBufferSize = 1024;
	// uint8 udpBuffer[MaxUdpBufferSize]; //received udp buffer
	// uint8 sendBuffer[104]; //received udp buffer
	bool vehinfoup1 = false, vehinfoup2 = false;
	CanFrame CanFrame_sent;
	//vector<TimestampedCanFrame> frames;

	uint8 brake_status1 = 0;
	veh_info dataout;



	//CanFrame Can_tosent;

	//dengnanshan
	//input
	NavSatFix gps_data;
	//icvImu icv_Imu_data;
	Imu imu_data;
	// out put
	double strangle;
	data::icvFloat32Data *outvalue;

	bool load_aps_map = false;
	bool havestart = false;
    bool have_trag = false;
	float xx_t;
	float yy_t;
	float xx_aps;
	float yy_aps;
	//ofstream ftestx;
	//ofstream ftesty("yaps.txt", ios::app);
	ifstream bestLmap;   //("/home/icv-00/Desktop/aps/2019_3_30/catkin_ws_0330ceshi/src/aps/bestL20_20_100_100_30.txt");
	ifstream BezierKmap; //("/home/icv-00/Desktop/aps/2019_3_30/catkin_ws_0330ceshi/src/aps/Bezier20_20_100_100_30.txt");
	string bestLmap_ads = "/home/icv-00/Desktop/aps/2019_3_30/catkin_ws_0330ceshi/src/aps/bestL20_20_100_100_30.txt";
	string BezierKmap_ads = "/home/icv-00/Desktop/aps/2019_3_30/catkin_ws_0330ceshi/src/aps/Bezier20_20_100_100_30.txt";
	float bestL[100][100][29];
	float BezierK[100][100][29];
	float ArcKMap[100][100][29];
	float trsangele = 0;
	//float delta_T = 1.0 / 5.0;
	float ACC_v = 0;
	float wheel_angle_vehicle;
	int vehicle_wheel_dre;
	float vehicle_wheel_speed;
	float yaw_speed;
	int yaw_dre;
	float yaw_angle;
	float predict_angle;
	float ControlPoint[2][3];
	float ControlPoint2[8];
	std::vector<RoadPoint> aps_points;
	RoadPoint pk_Point;
	Car_Status car_Status;
	Car_Status startPoint;
	Car_Status trans_p;
	int control = 0;
	float wheel_angle_degree_max = 470;
	float wheel_angle_degree_sec = 400;
	float wheel_now;
	float wheel_last = 0;
	float wheel_angle_degree;
	float lon[102400];
	float lat[102400];
	float lat_b;
	float lon_b;
	int index_i = 0;
	float scale[2];
	float xx[102400];
	float yy[102400];
	float wheelout[10];
	float a;
	float f;
	float b;
	float e2;
	float A;
	float B;
	double angle_imu;
	//float lon_temp;
};
ICV_REGISTER_FUNCTION(Xiaopengaps)

#endif //
