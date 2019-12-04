#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include <visualization_msgs/Marker.h>
#include "sensor_msgs/NavSatFix.h" 
//#include "agitr/vehicleout.h"
#include <fstream>
using namespace std;

int main(int argc,char **argv)
{       
    float path_x[3000];   //路径x坐标
    float path_y[3000];   //路径y坐标
    int count=0;           //用于存储路径
    float i_20_1=0;
    float i_20_2=1;
    ros::init(argc, argv, "plot_path");
    ros::NodeHandle n;
    //ros::Publisher pub_path=nh.advertise<agitr::vehicleout>("plotpath",1000);
    ros::Publisher path_pub = n.advertise<visualization_msgs::Marker>("plotpath", 10);
    //ros::Publisher path_pub_mkr = n.advertise<visualization_msgs::Marker>("path_marker_mkr", 10);
    ifstream ifile1("/home/icv002/Desktop/lane_data/x.txt");
    ifstream ifile2("/home/icv002/Desktop/lane_data/y.txt");
    //agitr::vehicleout path;
    ros::Rate r(100);
    visualization_msgs::Marker pathpoint;                         //路径
    pathpoint.header.frame_id= "/my_frame";
    pathpoint.header.stamp =ros::Time::now();
    pathpoint.ns = "pathpoint";
    pathpoint.action = visualization_msgs::Marker::ADD;
    pathpoint.pose.orientation.w = 1.0;
    pathpoint.id = 4;
    pathpoint.type = visualization_msgs::Marker::POINTS;
  
       // POINTS markers use x and y scale for width/height respectively
    pathpoint.scale.x = 0.2;
    pathpoint.scale.y = 0.2;
   
       // Points are green
    pathpoint.color.r = 1.0;
    pathpoint.color.a = 1.0;

    geometry_msgs::Point pat;

    while (ros::ok())
    { 
      if ((ifile1.good())&&(ifile2.good()))
        {
	 ifile1>>path_x[count];
	 ifile2>>path_y[count];
	 ROS_INFO("x %f",path_x[count]);
	 ROS_INFO("y %f",path_y[count]);
         pat.x=path_x[count];
         pat.y=path_y[count];
         pat.z=0;
         pathpoint.points.push_back(pat);      //预瞄点线
         path_pub.publish(pathpoint);
        count=count+1;
         }
     
      r.sleep();
    }
      return 0;

}
