#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "sensor_msgs/NavSatFix.h" 
#include "agitr/vehicleout.h"
#include <cmath>
 
float vehicle_x;
float vehicle_y;
float preview_x;
float preview_y;
float path_x;
float path_y;
float error_d;
int ii=0;
void positionrvizCallback(const agitr::vehicleout::ConstPtr& position)     //自车位置及预瞄点位置
     {
      ii=ii+1;
      //ROS_INFO("ii:%d",ii);
      vehicle_x=position->x_vehicle_gps;   //实际车辆位置
      //ROS_INFO("x_rec:%lf",x_rec);
      vehicle_y=position->y_vehicle_gps;
      preview_x=position->x_preview;   //预瞄点位置
      preview_y=position->y_preview;
      error_d=position->d_dis;
      //error_d=0.01*ii;
      //ROS_INFO("y_rec:%lf",y_rec);
      }

void pathCallback(const agitr::vehicleout::ConstPtr& path)     //自车位置及预瞄点位置
     {
      //ROS_INFO("ii:%d",ii);
      path_x=path->x_path;
      path_y=path->y_path;
      ROS_INFO("path_x:%lf",path_x);
      /*vehicle_y=position->y_vehicle;
      preview_x=position->x_re;   //预瞄点位置
      preview_y=position->y_re;
      error_d=position->d_dis;*/
      //error_d=0.01*ii;
      //ROS_INFO("y_rec:%lf",y_rec);
      }



 int main( int argc, char** argv )
   {
     ros::init(argc, argv, "points_and_lines");
     ros::NodeHandle n;
     ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
     ros::Publisher marker_pub2 = n.advertise<visualization_msgs::Marker>("path_marker", 10);
     ros::Subscriber sub = n.subscribe("y_draw", 1000, positionrvizCallback);          //车辆自车位置及预瞄点位置
     //ros::Subscriber sub_path = n.subscribe("plotpath", 1000, pathCallback);          //理论轨迹及预瞄点位置
     
     ros::Rate r(100);

     //float f = 0.0;
       visualization_msgs::Marker preview;                      //预瞄点，加上误差
       visualization_msgs::MarkerArray array,array_text;
       preview.header.frame_id= "/my_frame";
       preview.header.stamp =ros::Time::now();
       preview.ns = "preview_points";
       preview.action = visualization_msgs::Marker::ADD;
       preview.pose.orientation.w = 1.0;
       preview.id = 0;
       preview.type = visualization_msgs::Marker::LINE_LIST;
  
       // POINTS markers use x and y scale for width/height respectively
       preview.scale.x = 0.1;
   
       //points.text = "blablabla";
   
       // Points are green
       preview.color.g = 1.0f;
       preview.color.a = 1.0;

       visualization_msgs::Marker vehicle;                         //自车位置
       vehicle.header.frame_id= "/my_frame";
       vehicle.header.stamp =ros::Time::now();
       vehicle.ns = "vehicle_position";
       vehicle.action = visualization_msgs::Marker::ADD;
       vehicle.pose.orientation.w = 1.0;
       vehicle.id = 1;
       vehicle.type = visualization_msgs::Marker::POINTS;
  
       // POINTS markers use x and y scale for width/height respectively
       vehicle.scale.x = 0.2;
       vehicle.scale.y = 0.2;
   
       // Points are green
       vehicle.color.b = 1.0;
       vehicle.color.a = 1.0;


       visualization_msgs::Marker preview_point;                         //自车位置
       preview_point.header.frame_id= "/my_frame";
       preview_point.header.stamp =ros::Time::now();
       preview_point.ns = "preview_points";
       preview_point.action = visualization_msgs::Marker::ADD;
       preview_point.pose.orientation.w = 1.0;
       preview_point.id = 1;
       preview_point.type = visualization_msgs::Marker::POINTS;
  
       // POINTS markers use x and y scale for width/height respectively
       preview_point.scale.x = 0.2;
       preview_point.scale.y = 0.2;
   
       // Points are green
       preview_point.color.g = 1.0;
       preview_point.color.a = 1.0;
       
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

       //geometry_msgs::Point pre;
       geometry_msgs::Point vhl;
       geometry_msgs::Point ptxt;
       geometry_msgs::Point pre;
       geometry_msgs::Point pat;
       char strTmp[90];
      while (ros::ok())
      { 
       ros::spinOnce();
      
       pre.x=preview_x;        //预瞄点
       pre.y =preview_y;        
       //pre.x=vehicle_x;
       //pre.y=vehicle_y;
       pre.z = 0;    
       vhl.x=vehicle_x;       //车辆位置 
       vhl.y =vehicle_y;
       vhl.z = 0;
       

       pat.x=path_x;
       pat.y=path_y;
       pat.z=0;
       preview.points.push_back(pre);      //预瞄点线
       preview_point.points.push_back(pre);   //预瞄点位置
       //pathpoint.points.push_back(pat);      //预瞄点线
       vehicle.points.push_back(vhl);   //    车辆位置
       pre.z += error_d;
       preview.points.push_back(pre);

       //preview.lifetime = ros::Duration();
       //vehicle.lifetime = ros::Duration();

       marker_pub.publish(preview);       //预瞄点+误差线
       marker_pub.publish(preview_point);
       marker_pub.publish(vehicle);
       //marker_pub2.publish(pathpoint);
       r.sleep();

    }
    preview.points.clear();
    preview_point.points.clear();
    vehicle.points.clear();
  }
