#include "fusion.h"
#include "sensor_msgs/Imu.h"  
#include "sensor_msgs/NavSatFix.h" 
#include "agitr/veh_pos.h"
#include <fstream>
#include <math.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/DenseCoeffsBase.h>
#define PI 3.1415926
#define pi 3.1415926
using namespace std;
using namespace Eigen;

//用来接收实时的数据并进行转化
double lon[102400];
double lat[102400];
double lat_b;
double lon_b;
int index_i=0;
double scale[2];
float xx[102400];
float yy[102400];
double a;
double f;
double b;
double e2;
double A;
double B;
double angle_imu;

double lon_temp;
double lat_temp;
//航向角
Quaternionf q;	//wxyz
struct Imu_Data{
double angle;
double x;
double y;
double z;

float imu_angle_ori; // 偏东-1　偏西1
uint8 flag_num:1;
};
Imu_Data obj_imu_data;
float temp_imu_angle_ori = 0;

ofstream ftestx("x.txt",ios::app);   //ios::app 从文件末尾开始写，防止丢失文件中原来就有的内容
ofstream ftesty("y.txt",ios::app);   //ios::app 从文件末尾开始写，防止丢失文件中原来就有的内容

//void imu_orientation(double lon_temp,double lon);
void imu_orientation(double lon_temp,double lon ,double lat_temp,double lat);
void lon_alt2xyzCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)         //订阅到话题数据并进行转换为xyz
{
 /*index_i=index_i+1;                          //接收一个数据转换一个数据
 lat=msg->latitude;
 lon=msg->longitude;

 if (index_i<500)
    {
     xx[index_i]=msg->latitude;
     yy[index_i]=msg->longitude;
     lat_b=xx[1];                               //把车辆起点当成坐标远点
     lon_b=yy[1];
     }
 if (index_i<10)
    {
     a=6378137.0;
     f=298.2572; 
     b=(f - 1) / f * a;
     e2=(a*a - b*b) / (a*a);
     A = a * (1 - e2) / pow((1-e2*pow(sin(lat_b/180.0*PI),2)),1.5);
     B = a * cos(lat_b/180.0*PI)/sqrt(1-e2*pow(sin(lat_b/180.0*PI),2));
     scale[1]=B*1.0/180.0*PI;
     scale[2]=A*1.0/180.0*PI;
      }
  x=(lon - lon_b) * scale[1];   //转换为坐标
  y=(lat - lat_b) * scale[2];*/
  index_i=index_i+1;                          //接收一个数据转换一个数据
  lat[index_i]=msg->latitude;
  lon[index_i]=msg->longitude;
  //lat[1]=40.0000974393;
  //lon[1]=116.330248026;  meishuguan

  lat[1]=40.0068589313;
  lon[1]=116.327951572;  //qiyansou
 if (index_i<3)
    {
     a=6378137.0;
     f=298.2572; 
     b=(f - 1) / f * a;
     e2=(a*a - b*b) / (a*a);
     A = a * (1 - e2) / pow((1-e2*pow(sin(lat[1]/180.0*pi),2)),1.5);
     B = a * cos(lat[1]/180.0*pi)/sqrt(1-e2*pow(sin(lat[1]/180.0*pi),2));
     scale[1]=B*1.0/180.0*pi;
     scale[2]=A*1.0/180.0*pi;
      }
  xx[index_i]=(lon[index_i] - lon[1]) * scale[1];   //转换为坐标
  yy[index_i]=(lat[index_i] - lat[1]) * scale[2];
   
  ROS_INFO("lon_b [%f]",lon[1]); 
  ROS_INFO("lat_b [%f]",lat[1]);                              
  ftestx <<xx[index_i]<<endl;                       //转换后写入txt中
  ftesty <<yy[index_i]<<endl;
  ROS_INFO("xxx:%f",xx[index_i]);
  ROS_INFO("yyy:%f",yy[index_i]);


  ROS_INFO("lon:%f",lon[index_i]);
  ROS_INFO("lat:%f",lat[index_i]);                              
  //imu_orientation(lon_temp, lon[index_i]);
  imu_orientation(lon_temp, lon[index_i], lat_temp, lat[index_i]);           //计算
}

void imu_data_callback(const sensor_msgs::Imu::ConstPtr& msg) 
{ 
    q.w() = msg->orientation.w; 
    q.x() = msg->orientation.x;
    q.y() = msg->orientation.y;
    q.z() = msg->orientation.z;
	//ROS_INFO("msg->orientation.w:%lf",q.w());
    
}
int main(int argc, char **argv)  
{ 
  ros::init(argc, argv, "GPS_IMU");  
  ros::NodeHandle nxy;  
  ros::Subscriber sub_gps_fix = nxy.subscribe("/gps/fix",1,lon_alt2xyzCallback);    //订阅到话题数据
  ros::Subscriber sub_imu_data = nxy.subscribe("imu/data",1, imu_data_callback);  


  ros::Publisher chatter_pub = nxy.advertise<agitr::veh_pos>("vehicle_position", 1000);   //把转换过得xy坐标发出来
  ros::spinOnce();         //消除丢帧问题
  agitr::veh_pos pos;
  
  ros::Rate loop_rate(5); 
  //ros::Rate loop_rate(100); 
  while (ros::ok())
     {
      ros::spinOnce(); 
      pos.pos_x=xx[index_i];
      pos.pos_y=yy[index_i];
      //pos.orientation_angle = obj_imu_data.angle+90;

      if (( obj_imu_data.angle>=-180)&& (obj_imu_data.angle<-90))   //
           {angle_imu= obj_imu_data.angle+90;}

      else if (( obj_imu_data.angle>=-90)&& (obj_imu_data.angle<0))   //
               {angle_imu= obj_imu_data.angle+90;}

            else if (( obj_imu_data.angle>=0)&& (obj_imu_data.angle<90))   //
               {angle_imu= obj_imu_data.angle+90;}
                  else
                     {angle_imu= obj_imu_data.angle-270;}
     pos.orientation_angle = angle_imu;
     //pos.imu_angle=obj_imu_data.angle;

      chatter_pub.publish(pos); 
      loop_rate.sleep();
      } 
  return 0;
  ftestx.close();       //关闭文件
  ftesty.close();
}

void imu_orientation(double lon_temp_m,double lon_m, double lat_temp_m, double lat_m){

    double lon_last = lon_temp_m;
	double dev_lon = lon_last - lon_m;

    double lat_last = lat_temp_m;
	double dev_lat = lat_last - lat_m;
    //zhengfu
	if( (dev_lat>0 && dev_lon>0) || (dev_lat<0 && dev_lon>0) ){
		obj_imu_data.imu_angle_ori = -1;
        temp_imu_angle_ori = obj_imu_data.imu_angle_ori;
        obj_imu_data.flag_num = 1;
	}
	else if( (dev_lat>=0 && dev_lon<0) || (dev_lat<0 && dev_lon<0) ){
        obj_imu_data.imu_angle_ori = 1;
        temp_imu_angle_ori = obj_imu_data.imu_angle_ori;
        obj_imu_data.flag_num = 1;
    }
    // else if(  (dev_lon==0 && dev_lat==0)
    //         ||(dev_lon!=0 && dev_lat==0)
    //         ||(dev_lon==0 && dev_lat!=0) ){
    else{
        obj_imu_data.imu_angle_ori = temp_imu_angle_ori;
        if(obj_imu_data.flag_num>4){
            obj_imu_data.imu_angle_ori = 1;
            obj_imu_data.flag_num = 1;
        }
        obj_imu_data.flag_num++;
    }
	obj_imu_data.angle = (obj_imu_data.imu_angle_ori)*acos(q.w())*360/PI;
// 判断
    // if(obj_imu_data.angle>176 || obj_imu_data.angle<-176){
    //     obj_imu_data.angle = 180.000000;
    // }
    // else if(obj_imu_data.angle>-4 && obj_imu_data.angle<4){
    //     obj_imu_data.angle = 0.000000;
    // }
    // else{
    //     obj_imu_data.angle = obj_imu_data.angle;
    // }

    double Theta = obj_imu_data.angle;
	double sin_theta = sin(Theta*PI/360);
	obj_imu_data.x = q.x()/sin_theta;
	obj_imu_data.y = q.y()/sin_theta;
	obj_imu_data.z = q.z()/sin_theta;

	ROS_INFO("obj_imu_data.imu_angle_ori:%f",obj_imu_data.imu_angle_ori);
	ROS_INFO("Imu_Data.angle:%lf",obj_imu_data.angle);

	lon_temp = lon_m;
    lat_temp = lat_m;
}
