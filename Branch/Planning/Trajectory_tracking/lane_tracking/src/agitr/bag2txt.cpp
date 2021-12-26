#include "ros/ros.h"  
#include "geometry_msgs/Twist.h" 
#include <fstream>
#include "agitr/vehicleout.h"
using namespace std;

//可以用来记录话题上数据并写到txt文件中

ofstream ftest("xvehicle.txt",ios::app);   //ios::app 从文件末尾开始写，防止丢失文件中原来就有的内容
ofstream ftesty("yvehicle.txt",ios::app);   //ios::app 从文件末尾开始写，防止丢失文件中原来就有的内容
ofstream ftestprex("xpreview.txt",ios::app);   //ios::app 从文件末尾开始写，防止丢失文件中原来就有的内容
ofstream ftestprey("ypreview.txt",ios::app);   //ios::app 从文件末尾开始写，防止丢失文件中原来就有的内容
ofstream ftesttheta("thetavehicle.txt",ios::app); 
ofstream ftestd("d.txt",ios::app);   //ios::app 从文件末尾开始写，防止丢失文件中原来就有的内容

float vehicle_x;
float vehicle_y;
float preview_x;
float preview_y;
float path_x;
float path_y;
float error_d;
float vehicletheta;
/*void bag2txtCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
 x=msg->linear.x;
 ftest <<x<<endl;
 ROS_INFO("%f",x);
}*/

void positionrvizCallback(const agitr::vehicleout::ConstPtr& position)     //自车位置航向角及预瞄点位置,误差，都是从y_draw话题上听到的数据
     {
      vehicle_x=position->x_vehicle;              //车辆位置
      ftest <<vehicle_x<<endl;
      ROS_INFO("vehicle_x:%lf",vehicle_x);
      vehicle_y=position->y_vehicle;
      ftesty<<vehicle_y<<endl;
      ROS_INFO("vehicle_y:%lf",vehicle_y);
      preview_x=position->x_preview;   //预瞄点位置
      ftestprex<<preview_x<<endl;
      preview_y=position->y_preview;
      ftestprey<<preview_y<<endl;
      error_d=position->d_dis;     //预瞄误差
      ftestd<<error_d<<endl;
      vehicletheta=position->vehicle_theta;          //车辆航向角
      ftesttheta<<vehicletheta<<endl;

      }



int main(int argc, char **argv)  
{ 
  ros::init(argc, argv, "bag2txt");  
  ros::NodeHandle n;  
  ros::Subscriber sub = n.subscribe("y_draw", 1000, positionrvizCallback); 
  //ros::Subscriber sub = n.subscribe("y_draw", 1000, positionrvizCallback); 
  ros::Rate loop_rate(10); 
  while (ros::ok())
     {
      ros::spinOnce(); 
      loop_rate.sleep();
      } 
  ftest.close();
}
