// @brief: Enter fusion
// objective: Collect all sensor data, enter the fusion module
// Author: Chunlei YU. Contact me at: yuchunlei@mail.tsinghua.edu.cn

#ifndef CAR_OBSF_ROS_H
#define CAR_OBSF_ROS_H

#include <iostream>
#include <fstream>

#include <eigen3/Eigen/Dense>
// #include <Eigen/Geometry>
#include "ros_fusion/DriveObsArray.h"
#include "ros_fusion/DriveObs.h"
#include "ros_fusion/DriveRadarObsArray.h"
#include "ros_fusion/DriveRadarObs.h"

#include "ros_fusion/ivsensorlrrobj.h"
#include "ros_fusion/radarlrrobject.h"

#include "ros_fusion/ivsensorsrrobj.h"
#include "ros_fusion/radarsrrobject.h"

#include "ros_fusion/track.h"
#include "ros_fusion/TrackArray.h"

#include "ros_fusion/BoundingBoxes.h"
#include "ros_fusion/BoundingBox.h"

#include "filters/obsf_uncertainty_fusion_engine.h"
#include "objects/obsf_header.h"
// #include "visualization/obsf_pcl_visual.h"

// #include <ros/ros.h>
// #include <ros/time.h>
#include <std_msgs/Header.h>
#include "std_msgs/String.h"
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/common/time.h>
//#include <pcl/common/transforms.h>

//#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/MarkerArray.h>

/*4lidar*/
#include "ros_fusion/ObjectArray.h"
#include "ros_fusion/Object.h"

namespace tsinghua{
namespace dias{
namespace fusion{

// ros::Publisher g_obs_publisher;/
// ros::Publisher g_obs_publisher_proto;/

ros::Publisher pub_fusion_markerArray;
ros::Publisher pub_fusion_text;

ros::Publisher pub_lidar_box;
ros::Publisher pub_lidar_velocity;
ros::Publisher pub_lidar_contour;
ros::Publisher pub_lidar_markerArray;
ros::Publisher pub_lidar_text;
ros::Publisher pub_camera_text;
ros::Publisher pub_camera_markerArray;
ros::Publisher pub_fusion_region;
ros::Publisher pub_box,pub_cam_bbox;
ros::Publisher pub_contour;
ros::Publisher pub_velocity, pub_imu_acceleration;
ros::Publisher pub_gps_route;
ros::Publisher pub_final_fusion;
ros::Publisher lane_pub;
ros::Publisher radar_raw_pub;



// tsinghua::dias::fusion::ObsfVisual _visualizer;
tsinghua::dias::fusion::OBSFUncertaintyFusionEngine _Engine;

Eigen::Affine3d transform; // cloud to radar transform

float lidar_1 = 100;
float lidar_2 = -100;
float glb_twist_linear_x = 0;
float glb_twist_linear_y = 0;
float glb_twist_linear_z = 0;
float course_angle_rad = 0;
float route_x = 0;
float route_y = 0;
int8_t index_gps = 0;
float glb_rviz_route_x = 0;
float glb_rviz_route_y = 0;
double lidar_time = -4;
double front_radar_time = -4;
double srr_radar_time = -4;
double now_time = -4;
double rear_lidar_time = -4;
double rear_radar_time = -4;
double rear_srr_time = -4;
double rear_now_time = -4;
float imu_roll = 0.0;
float imu_pitch = 0.0;
float imu_yaw = 0.0;
float imu_acceleration_x = 0.0;
float imu_acceleration_y = 0.0;

class Obsfusion{
public:
    Obsfusion(){}
    ~Obsfusion(){}

    static void obs_callback_lidar3d(const ros_fusion::DriveObsArray &msg){
 
    }

    static uint64_t unix2gps(uint64_t unix_time_usec) {
        // const uint64_t leapSeconds = 16;
        const uint64_t leapSeconds =
            unix_time_usec < uint64_t(1435708799) * 1e6 ? 16 : 17;
        int64_t gps = unix_time_usec - (uint64_t(315964800) - leapSeconds) * 1e6;
        return gps > 0 ? gps : 0;
    }

    static void judge_control_mode(bool location_sensor){

        double location_time;
        double location_radar_sensor;
        double location_srr_sensor;
        double location_lidar_sensor;

        if(location_sensor == 0){
            location_time = now_time;
            location_radar_sensor = front_radar_time;
            location_srr_sensor = srr_radar_time;
            location_lidar_sensor = lidar_time;
        }else{
            location_time = rear_now_time;
            location_radar_sensor = rear_radar_time;
            location_srr_sensor = rear_srr_time;
            location_lidar_sensor = rear_lidar_time;
        }
        int temp_control_mode;
        bool flag[3]; //lost sensors' flag will be set to 1
        for(int i=0;i<3;i++){
            flag[i]=0;
        }
        // std::cout << "location_time: " << location_time << std::endl;
        // std::cout << "location_lidar_sensor: " << location_lidar_sensor << std::endl;
        // std::cout << "location_radar_sensor: " << location_radar_sensor << std::endl;
        // std::cout << "location_srr_sensor: " << location_srr_sensor << std::endl;

        if(location_time - location_radar_sensor > 2){flag[0]=1;}
        if(location_time - location_srr_sensor > 2){flag[1]=1;}
        if(location_time - location_lidar_sensor > 2){flag[2]=1;}

        if(location_time - location_radar_sensor < 0){
            if(location_sensor == 0 ){
                front_radar_time=location_time;
            }else{
                rear_radar_time=location_time;
            }
        }
        if(location_time - location_srr_sensor < 0){
            if(location_sensor == 0 ){
                srr_radar_time=location_time;
            }else{
                rear_srr_time=location_time;
            }
        }
        if(location_time - location_lidar_sensor < 0){
            if(location_sensor == 0 ){
                lidar_time=location_time;
            }else{
                rear_lidar_time=location_time;
            }
        }

        if(flag[0]+flag[1]+flag[2] == 2){
            temp_control_mode = -1;
        }
        else if(flag[0]+flag[1]+flag[2] == 0){
            temp_control_mode = 255;
        }else if(flag[0]+flag[1]+flag[2] == 3){
            temp_control_mode = 111;
        }else{
            if (flag[0] == 1){
                temp_control_mode = 0;
            }
            if (flag[1] == 1){
                temp_control_mode = 1;
            }
            if (flag[2] == 1){
                temp_control_mode = 6;
            }
        }
        _Engine.set_control_mode(temp_control_mode, location_sensor);
    }
    static void Data_Projection(tsinghua::dias::fusion::OBSFObs3d &obs_tmp)
    {   
        // ROS_INFO("Data_Projection");
        double c= 0.66;
        // double omega = _Vec_lrrData[i].x *0.000358601392 + _Vec_lrrData[i].y *0.000001777188 + 0.001096276284065;
        // _Vec_lrrData[i].x_pixel= (_Vec_lrrData[i].x *0.2313509116 + _Vec_lrrData[i].y * 0.285860720217 + 0.711312562843886) / omega /1324* 768;
        // _Vec_lrrData[i].y_pixel= (_Vec_lrrData[i].x *0.123527788805 + _Vec_lrrData[i].y * 0.006763422328 + 0.702874871918316) / omega /828 * 480;
        double omega =(obs_tmp._obs_position._x-LRR_F_pos_x) *-0.00059703 +obs_tmp._obs_position._y *-0.000050721 + c*0.00010254-0.0016;
        obs_tmp._v_center= ((obs_tmp._obs_position._x-LRR_F_pos_x)*-0.22384 + obs_tmp._obs_position._y* 0.24574 +c*0.045906 -0.5648) / omega ;
        obs_tmp._u_center= ((obs_tmp._obs_position._x-LRR_F_pos_x) * -0.11899 +obs_tmp._obs_position._y * -0.026076 +c*0.28690-0.8252) / omega ;       
        // ROS_INFO("ID=%d", i);
        // ROS_INFO("x_pixel=%f", _Vec_lrrData[i].x_pixel);
        // ROS_INFO("y_pixel=%f", _Vec_lrrData[i].y_pixel);
        // ROS_INFO("x=%f", _Vec_lrrData[i].x);
        // ROS_INFO("y=%f", _Vec_lrrData[i].y);
        // ROS_INFO(" ");
    }
        /* -0.22384 0.24574 0.045906 -0.5648
        -0.11899 -0.026076 0.28690 -0.8252
        -0.00059703 -0.000050721 0.00010254 -0.0016 */

    static void obs_callback_camera_bbox_online(const ros_fusion::BoundingBoxes &_objects){
        // std::cout << "Now we enter the bbox callback" << std::endl;
        now_time = _objects.header.stamp.toSec();
        judge_control_mode(0);

        uint64_t unix_time_usec = static_cast<uint64_t>(_objects.header.stamp.toSec() * 1e6);
        uint64_t gps_time_usec = unix2gps(unix_time_usec);
        uint64_t sec =  gps_time_usec / 1000000;
        uint64_t nsec =  (gps_time_usec % 1000000) * 1000;


        Eigen::Matrix4d radar_pose = Eigen::Matrix4d::Identity();

        tsinghua::dias::fusion::OBSFHeader header;
        header._seq = _objects.header.seq;
        // header._stamp._sec = msg.header.stamp.sec;
        header._stamp._sec = sec;
        // header._stamp._nsec = msg.header.stamp.nsec;
        header._stamp._nsec = nsec;
        header._frame_id = _objects.header.frame_id;           

        tsinghua::dias::fusion::OBSFObs3dFrame obs_3d_frame;
        obs_3d_frame.set_header(header);        

        std::vector<tsinghua::dias::fusion::OBSFObs3d> obs_3d_obstacles;
	    std::set<int> current_ids;
        tsinghua::dias::fusion::OBFSVector3d point;

        for (size_t i = 0; i < _objects.boundingBoxes.size(); i++)
        {
            tsinghua::dias::fusion::OBSFObs3d obs_3d;
            obs_3d._header._seq = _objects.header.seq;
            obs_3d._header._stamp._sec = sec;
            obs_3d._header._stamp._nsec = nsec;
            obs_3d._header._frame_id = _objects.header.frame_id;
            //7.7 add sensor header for big fusion   by kiki
            obs_3d._sensor_header =  _objects.header;
            obs_3d._obs_id = 1111;
            obs_3d.flag_repeat = 0;
             
            obs_3d._sensor_type = 7;

            obs_3d.rangex_rms = 100.00f;                
            obs_3d.speedx_rms = 100.00f;                
            obs_3d.rangey_rms = 100.00f;
            obs_3d.speedy_rms = 100.00f;
            obs_3d.rangez_rms = 100.00f; 
            obs_3d.speedz_rms = 100.00f;
            obs_3d.ori_rms = 21.00f;
            obs_3d.oriRate_rms = 21.00f;
            float beta = (_objects.boundingBoxes[i].ori+PI/2);
            if(beta>2*PI){    beta = beta-2*PI;    }
            if(beta<=PI){    beta = -beta;    
            }else{    beta = 2*PI - beta;    }
            obs_3d._obs_theta = beta;
            obs_3d._Box3d.x = _objects.boundingBoxes[i].dz; //x=length y=width z=height
            obs_3d._Box3d.y= _objects.boundingBoxes[i].dx;
            obs_3d._Box3d.z = _objects.boundingBoxes[i].dy;
            // obs_3d._obs_theta = _objects.boundingBoxes[i].ori;
    //0x0:point, 0x1:car, 0x2:truck, 0x3:pedestrian, 0x4:motorcycle, 0x5:bicycle, 0x6:wide, 0x7:reserved  0x8:dog
            if(_objects.boundingBoxes[i].Class=="person")
            {
            obs_3d._object_type=0x3;
            }
            else if (_objects.boundingBoxes[i].Class=="motorcycle")
            {
            obs_3d._object_type=0x4;
            }
            else if (_objects.boundingBoxes[i].Class=="bicycle")
            {
            obs_3d._object_type=0x5;
            }
            else if (_objects.boundingBoxes[i].Class=="car" || _objects.boundingBoxes[i].Class=="van")
            {
            obs_3d._object_type=0x1;
            }
            else if (_objects.boundingBoxes[i].Class=="truck")
            {
            obs_3d._object_type=0x2;
            }
            else if (_objects.boundingBoxes[i].Class=="dog")
            {
            obs_3d._object_type=0x8;
            }
            else
            {
            obs_3d._object_type=0x7;
            }
            obs_3d._v_center=(_objects.boundingBoxes[i].xmax+_objects.boundingBoxes[i].xmin)/2;
            obs_3d._u_center=(_objects.boundingBoxes[i].ymax+_objects.boundingBoxes[i].ymin)/2;
            obs_3d._width_pixel=_objects.boundingBoxes[i].xmax-_objects.boundingBoxes[i].xmin;
            obs_3d._height_pixel=_objects.boundingBoxes[i].ymax-_objects.boundingBoxes[i].ymin;


      /* -0.22384 0.24574 0.045906 -0.5648
        -0.11899 -0.026076 0.28690 -0.8252
        -0.00059703 -0.000050721 0.00010254 -0.0016 */
            Eigen::Matrix<float,3,3> HH;
            float point_z = 0;
            HH <<  -0.11899,-0.026076,0.28690*point_z-0.8252,
                   -0.22384,0.24574,0.045906*point_z-0.5648,
                   -0.00059703,-0.000050721,0.00010254*point_z-0.0016;

            Eigen::Matrix<float,3,3> HH_1=HH.inverse();
            double bottom_u = _objects.boundingBoxes[i].ymax;
            double bottom_v = (_objects.boundingBoxes[i].xmax+_objects.boundingBoxes[i].xmin)/2;
            float xy = HH_1(2,0)*bottom_u+HH_1(2,1)*bottom_v+HH_1(2,2);
            // std::cout << "[+]camera: bottom_uv "<<  bottom_u<<" "<<bottom_v << std::endl;
            // std::cout << "[+]camera: xy "<<  xy << std::endl;

            //  std::cout << HH_1 <<std::endl;
            float a = _objects.boundingBoxes[i].theta;
            obs_3d._obs_position._x = _objects.boundingBoxes[i].distance_x;
            obs_3d._obs_position._y = _objects.boundingBoxes[i].distance_y;
            std::cout << "[+]camera: ipm x "<< (HH_1(0,0)*bottom_u+HH_1(0,1)*bottom_v+HH_1(0,2))/xy + LRR_F_pos_x << std::endl;
            std::cout << "[+]camera: ipm y "<< (HH_1(1,0)*bottom_u+HH_1(1,1)*bottom_v+HH_1(1,2))/xy+LRR_F_pos_y<< std::endl;

            //  std::cout << "[+]camera: LRR_F_pos_x "<<LRR_F_pos_x<< std::endl;
            obs_3d._obs_position._x = (HH_1(0,0)*bottom_u+HH_1(0,1)*bottom_v+HH_1(0,2))/xy + LRR_F_pos_x;
            obs_3d._obs_position._y = (HH_1(1,0)*bottom_u+HH_1(1,1)*bottom_v+HH_1(1,2))/xy+LRR_F_pos_y;
            obs_3d._obs_position._z = 0.66f;
            obs_3d._velocity._x = 0;
            obs_3d._velocity._y = 0;
            obs_3d._velocity._z = a;
            obs_3d.speedx_rms = 100.00f;
            obs_3d.speedy_rms = 100.00f;
            obs_3d.speedz_rms = 100.00f;
          //  obs_3d._obs_position = point;
            obs_3d._acceleration.x = 0;
            obs_3d._acceleration.y = 0;
            obs_3d._self_velocity._x = glb_twist_linear_x*cosf(course_angle_rad)+glb_twist_linear_y*sinf(course_angle_rad);
            obs_3d._self_velocity._y = glb_twist_linear_y*cosf(course_angle_rad)-glb_twist_linear_x*sinf(course_angle_rad);
            obs_3d._self_velocity._z = glb_twist_linear_z;
            obs_3d._course_angle_rad = course_angle_rad;
            std::cout << "[+]camera: x"<<  obs_3d._obs_position._x << std::endl;
            std::cout << "[+]camera: y"<<  obs_3d._obs_position._y << std::endl;
            
         //   obs_3d.FromWhichSensor.push_back(1);

            obs_3d_obstacles.push_back(obs_3d);
        }
        // fusion_raw_data_visualization(obs_3d_obstacles);  
        if (obs_3d_obstacles.size() != 0){
            obs_3d_frame.set_obstacles(obs_3d_obstacles);
            _Engine.process_radar_frame(obs_3d_frame);
            publish_final_fusion();
            fusion_visualization();
        }
    }

    static void obs_callback_local_front_lidar_online(const ros_fusion::ObjectArray &msg) {

        // std::cout << "Now we enter the front lidar callback" << std::endl;

        lidar_time = msg.header.stamp.toSec();
        now_time = lidar_time;
        judge_control_mode(0);

        uint64_t unix_time_usec = static_cast<uint64_t>(msg.header.stamp.toSec() * 1e6);
        uint64_t gps_time_usec = unix2gps(unix_time_usec);
        uint64_t sec =  gps_time_usec / 1000000;
        uint64_t nsec =  (gps_time_usec % 1000000) * 1000;

        //Eigen::Matrix4d radar_pose = Eigen::Matrix4d::Identity();
        // now, the transform is from the radar point to cloud coordinate
        Eigen::Matrix4d radar_pose = transform.matrix();    
        tsinghua::dias::fusion::OBSFHeader header;
        header._seq = msg.header.seq;
        // header._stamp._sec = msg.header.stamp.sec;
        header._stamp._sec = sec;
        // header._stamp._nsec = msg.header.stamp.nsec;
        header._stamp._nsec = nsec;
        header._frame_id = msg.header.frame_id;           

        tsinghua::dias::fusion::OBSFObs3dFrame obs_3d_frame;
        obs_3d_frame.set_header(header);        

        std::vector<tsinghua::dias::fusion::OBSFObs3d> obs_3d_obstacles;
        std::set<int> current_ids;

        for (size_t i = 0; i < msg.objects.size(); i++)
        {
            if(msg.objects[i].object_box_size.x < 5 && msg.objects[i].object_box_size.y < 5){

                tsinghua::dias::fusion::OBSFObs3d obs_3d;
                obs_3d._header._seq = msg.header.seq;
                obs_3d._header._stamp._sec = sec;
                obs_3d._header._stamp._nsec = nsec;
                obs_3d._header._frame_id = msg.header.frame_id;
                obs_3d._sensor_header =  msg.header;
                obs_3d._obs_id = msg.objects[i].id;
                obs_3d.flag_repeat = 0;
                 
                obs_3d.rangex_rms = 7.00f;                
                obs_3d.speedx_rms = 100.00f;                
                obs_3d.accx_rms = 100.00f;
                obs_3d.rangey_rms = 7.00f;
                obs_3d.speedy_rms = 100.00f;
                obs_3d.accy_rms = 100.00f;
                obs_3d.rangez_rms = 7.00f; 
                obs_3d.speedz_rms = 100.00f;
                obs_3d.ori_rms = 100.00f;
                obs_3d.oriRate_rms = 100.00f;
                obs_3d._sensor_type = 6;
                obs_3d._object_type = 0;

                float _obs_x = msg.objects[i].object_box_center.pose.position.x;
                float _obs_y = msg.objects[i].object_box_center.pose.position.y;
                obs_3d._obs_position._x = _obs_x*cosf(LRR_F_ANGLE)+_obs_y*sinf(LRR_F_ANGLE)+LRR_F_pos_x-0.06;
                obs_3d._obs_position._y = _obs_y*cosf(LRR_F_ANGLE)-_obs_x*sinf(LRR_F_ANGLE)+LRR_F_pos_y+0.12;
                obs_3d._obs_position._z = 1;
                float _obs_vx = msg.objects[i].velocity.twist.linear.x;
                float _obs_vy = msg.objects[i].velocity.twist.linear.y;
                obs_3d._velocity._x = _obs_vx*cosf(LRR_F_ANGLE)+_obs_vy*sinf(LRR_F_ANGLE);
                obs_3d._velocity._y = _obs_vy*cosf(LRR_F_ANGLE)-_obs_vx*sinf(LRR_F_ANGLE);
                obs_3d._velocity._z = 0.0;
                float a,b;
                a, b, obs_3d._obs_theta = QuaternionToEulerAngles(msg.objects[i].object_box_center.pose.orientation);
                // std::cout<< "lidar_theta:  " << obs_3d._obs_theta*180/PI << std::endl;
                obs_3d.object_box_center = msg.objects[i].object_box_center;
                float ct_x = msg.objects[i].object_box_center.pose.position.x;
                float ct_y = msg.objects[i].object_box_center.pose.position.y;
                obs_3d.object_box_center.pose.position.x = ct_x*cosf(LRR_F_ANGLE) + ct_y*sinf(LRR_F_ANGLE) + LRR_F_pos_x-0.06;
                obs_3d.object_box_center.pose.position.y = ct_y*cosf(LRR_F_ANGLE) - ct_x*sinf(LRR_F_ANGLE) + LRR_F_pos_y+0.12;
                obs_3d.object_box_size = msg.objects[i].object_box_size;
                Data_Projection(obs_3d);
                for (int j = 0; j < msg.objects[i].contour_points.size(); j++)
                {
                    obs_3d.contour_points.push_back(msg.objects[i].contour_points[j]);
                    float c_x = obs_3d.contour_points[j].x;
                    float c_y = obs_3d.contour_points[j].y;
                    obs_3d.contour_points[j].x = c_x*cosf(LRR_F_ANGLE) + c_y*sinf(LRR_F_ANGLE) + LRR_F_pos_x-0.06;
                    obs_3d.contour_points[j].y = c_y*cosf(LRR_F_ANGLE) - c_x*sinf(LRR_F_ANGLE) + LRR_F_pos_y+0.12;
                }
                obs_3d._self_velocity._x = glb_twist_linear_x*cosf(course_angle_rad)+glb_twist_linear_y*sinf(course_angle_rad);
                obs_3d._self_velocity._y = glb_twist_linear_y*cosf(course_angle_rad)-glb_twist_linear_x*sinf(course_angle_rad);
                obs_3d._self_velocity._z = glb_twist_linear_z;
                obs_3d._course_angle_rad = course_angle_rad;
                obs_3d._acceleration.x = 0.0;
                obs_3d._acceleration.y = 0.0;
                obs_3d_obstacles.push_back(obs_3d);
            }
        }
        // fusion_raw_data_visualization(obs_3d_obstacles);           
        if(obs_3d_obstacles.size()!=0){
            obs_3d_frame.set_obstacles(obs_3d_obstacles);
            _Engine.process_radar_frame(obs_3d_frame);
            publish_final_fusion();
            fusion_visualization();
        }
    
    }

    static void obs_callback_local_rear_lidar_online(const ros_fusion::ObjectArray &msg) {

        // std::cout << "Now we enter the rear lidar callback" << std::endl;

        rear_lidar_time = msg.header.stamp.toSec();
        rear_now_time = rear_lidar_time;
        judge_control_mode(1);

        uint64_t unix_time_usec = static_cast<uint64_t>(msg.header.stamp.toSec() * 1e6);
        uint64_t gps_time_usec = unix2gps(unix_time_usec);
        uint64_t sec =  gps_time_usec / 1000000;
        uint64_t nsec =  (gps_time_usec % 1000000) * 1000;

        //Eigen::Matrix4d radar_pose = Eigen::Matrix4d::Identity();
        // now, the transform is from the radar point to cloud coordinate
        Eigen::Matrix4d radar_pose = transform.matrix();    
        tsinghua::dias::fusion::OBSFHeader header;
        header._seq = msg.header.seq;
        // header._stamp._sec = msg.header.stamp.sec;
        header._stamp._sec = sec;
        // header._stamp._nsec = msg.header.stamp.nsec;
        header._stamp._nsec = nsec;
        header._frame_id = msg.header.frame_id;           

        tsinghua::dias::fusion::OBSFObs3dFrame obs_3d_frame;
        obs_3d_frame.set_header(header);        

        std::vector<tsinghua::dias::fusion::OBSFObs3d> obs_3d_obstacles;
        std::set<int> current_ids;

        for (size_t i = 0; i < msg.objects.size(); i++)
        {
            if(msg.objects[i].object_box_size.x < 5 && msg.objects[i].object_box_size.y < 5){

                tsinghua::dias::fusion::OBSFObs3d obs_3d;
                obs_3d._header._seq = msg.header.seq;
                obs_3d._header._stamp._sec = sec;
                obs_3d._header._stamp._nsec = nsec;
                obs_3d._header._frame_id = msg.header.frame_id;
                obs_3d._sensor_header =  msg.header;
                obs_3d._obs_id = msg.objects[i].id;
                obs_3d.flag_repeat = 0;
                 

                obs_3d.rangex_rms = 7.00f;                
                obs_3d.speedx_rms = 100.00f;                
                obs_3d.accx_rms = 100.00f;
                obs_3d.rangey_rms = 7.00f;
                obs_3d.speedy_rms = 100.00f;
                obs_3d.accy_rms = 100.00f;
                obs_3d.rangez_rms = 7.00f; 
                obs_3d.speedz_rms = 100.00f;
                obs_3d.ori_rms = 100.00f;
                obs_3d.oriRate_rms = 100.00f;
                obs_3d._sensor_type = 8;
                obs_3d._object_type = 0;
                float _obs_x = msg.objects[i].object_box_center.pose.position.x;
                float _obs_y = msg.objects[i].object_box_center.pose.position.y;
                obs_3d._obs_position._x = _obs_x*cosf(LIDAR_B_ANGLE)+_obs_y*sinf(LIDAR_B_ANGLE)+LRR_B_pos_x-0.02;
                obs_3d._obs_position._y = _obs_y*cosf(LIDAR_B_ANGLE)-_obs_x*sinf(LIDAR_B_ANGLE)+LRR_B_pos_y-0.17;
                obs_3d._obs_position._z = 1;
                float _obs_vx = msg.objects[i].velocity.twist.linear.x;
                float _obs_vy = msg.objects[i].velocity.twist.linear.y;
                obs_3d._velocity._x = _obs_vx*cosf(LIDAR_B_ANGLE)+_obs_vy*sinf(LIDAR_B_ANGLE);
                obs_3d._velocity._y = _obs_vy*cosf(LIDAR_B_ANGLE)-_obs_vx*sinf(LIDAR_B_ANGLE);
                obs_3d._velocity._z = 0.0;
                float a,b;
                a, b, obs_3d._obs_theta = QuaternionToEulerAngles(msg.objects[i].object_box_center.pose.orientation);
                obs_3d.object_box_center = msg.objects[i].object_box_center;
                float ct_x = msg.objects[i].object_box_center.pose.position.x;
                float ct_y = msg.objects[i].object_box_center.pose.position.y;
                obs_3d.object_box_center.pose.position.x = ct_x*cosf(LIDAR_B_ANGLE) + ct_y*sinf(LIDAR_B_ANGLE) + LRR_B_pos_x-0.02;
                obs_3d.object_box_center.pose.position.y = ct_y*cosf(LIDAR_B_ANGLE) - ct_x*sinf(LIDAR_B_ANGLE) + LRR_B_pos_y-0.17;
                obs_3d.object_box_size = msg.objects[i].object_box_size;
                Data_Projection(obs_3d);
                for (int j = 0; j < msg.objects[i].contour_points.size(); j++)
                {
                    obs_3d.contour_points.push_back(msg.objects[i].contour_points[j]);
                    float c_x = obs_3d.contour_points[j].x;
                    float c_y = obs_3d.contour_points[j].y;
                    obs_3d.contour_points[j].x = c_x*cosf(LIDAR_B_ANGLE) + c_y*sinf(LIDAR_B_ANGLE) + LRR_B_pos_x-0.02;
                    obs_3d.contour_points[j].y = c_y*cosf(LIDAR_B_ANGLE) - c_x*sinf(LIDAR_B_ANGLE) + LRR_B_pos_y-0.17;
                }
                obs_3d._self_velocity._x = glb_twist_linear_x*cosf(course_angle_rad)+glb_twist_linear_y*sinf(course_angle_rad);
                obs_3d._self_velocity._y = glb_twist_linear_y*cosf(course_angle_rad)-glb_twist_linear_x*sinf(course_angle_rad);
                obs_3d._self_velocity._z = glb_twist_linear_z;
                obs_3d._course_angle_rad = course_angle_rad;
                obs_3d._acceleration.x = 0.0;
                obs_3d._acceleration.y = 0.0;
                obs_3d_obstacles.push_back(obs_3d);
            }

        }
        // fusion_raw_data_visualization(obs_3d_obstacles);           
        if(obs_3d_obstacles.size()!=0){
            obs_3d_frame.set_obstacles(obs_3d_obstacles);
            _Engine.process_radar_frame(obs_3d_frame);
            publish_final_fusion();
            fusion_visualization();
        }
    
    }
    static void obs_callback_local_front_radar_online(const ros_fusion::ivsensorlrrobj &msg) {

        // std::cout << "Now we enter the front radar callback" << std::endl;

        front_radar_time = msg.header.stamp.toSec();
        now_time = front_radar_time;
        judge_control_mode(0);

        uint64_t unix_time_usec = static_cast<uint64_t>(msg.header.stamp.toSec() * 1e6);
        uint64_t gps_time_usec = unix2gps(unix_time_usec);
        uint64_t sec =  gps_time_usec / 1000000;
        uint64_t nsec =  (gps_time_usec % 1000000) * 1000;

        //Eigen::Matrix4d radar_pose = Eigen::Matrix4d::Identity();
        // now, the transform is from the radar point to cloud coordinate
        Eigen::Matrix4d radar_pose = transform.matrix();            

        tsinghua::dias::fusion::OBSFHeader header;
        header._seq = msg.header.seq;
        // header._stamp._sec = msg.header.stamp.sec;
        header._stamp._sec = sec;
        // header._stamp._nsec = msg.header.stamp.nsec;
        header._stamp._nsec = nsec;
        header._frame_id = msg.header.frame_id;           

        tsinghua::dias::fusion::OBSFObs3dFrame obs_3d_frame;
        obs_3d_frame.set_header(header);        

        std::vector<tsinghua::dias::fusion::OBSFObs3d> obs_3d_obstacles;
        std::set<int> current_ids;
        OBFSVector3d absolute_vlty;
        for (size_t i = 0; i < msg.obs.size(); i++)
        {
            if(msg.obs[i].x>0){

                tsinghua::dias::fusion::OBSFObs3d obs_3d;
                obs_3d._header._seq = msg.header.seq;
                obs_3d._header._stamp._sec = sec;
                obs_3d._header._stamp._nsec = nsec;
                obs_3d._header._frame_id = msg.header.frame_id;
                obs_3d._sensor_header =  msg.header;
                obs_3d._obs_id = msg.obs[i].id;
                obs_3d.flag_repeat = 0;
                 
                obs_3d.rangex_rms = msg.obs[i].rangex_rms;                
                obs_3d.speedx_rms = msg.obs[i].speedx_rms;                
                obs_3d.accx_rms = msg.obs[i].accx_rms;
                obs_3d.rangey_rms = msg.obs[i].rangey_rms;
                obs_3d.speedy_rms = msg.obs[i].speedy_rms;
                obs_3d.accy_rms = msg.obs[i].accy_rms;
                obs_3d.rangez_rms = 7.00f; 
                obs_3d.speedz_rms = 100.00f;
                obs_3d._sensor_type = 0;
                obs_3d._object_type = 0;
                
                float _obs_x=msg.obs[i].x*cosf(LRR_F_ANGLE)+msg.obs[i].y*sinf(LRR_F_ANGLE);
                float _obs_y=msg.obs[i].y*cosf(LRR_F_ANGLE)-msg.obs[i].x*sinf(LRR_F_ANGLE);
                obs_3d._obs_position._x=_obs_x+LRR_F_pos_x;
                obs_3d._obs_position._y=_obs_y+LRR_F_pos_y;
                obs_3d._obs_position._z=1;
                obs_3d._velocity._x=msg.obs[i].speedx*cosf(LRR_F_ANGLE)+msg.obs[i].speedy*sinf(LRR_F_ANGLE);
                obs_3d._velocity._y=msg.obs[i].speedy*cosf(LRR_F_ANGLE)-msg.obs[i].speedx*sinf(LRR_F_ANGLE);
                obs_3d._velocity._z=0.0;
                obs_3d._self_velocity._x = glb_twist_linear_x*cosf(course_angle_rad)+glb_twist_linear_y*sinf(course_angle_rad);
                obs_3d._self_velocity._y = glb_twist_linear_y*cosf(course_angle_rad)-glb_twist_linear_x*sinf(course_angle_rad);
                obs_3d._self_velocity._z = glb_twist_linear_z;           
                absolute_vlty._x = obs_3d._velocity._x + obs_3d._self_velocity._x;
                absolute_vlty._y = obs_3d._velocity._y + obs_3d._self_velocity._y;
                if( sqrt(pow(absolute_vlty._x,2)+pow(absolute_vlty._y,2))>1 ){
                    obs_3d._obs_theta = atan2(absolute_vlty._y, absolute_vlty._x);
                    // std::cout << "[+]vy:"<<absolute_vlty._y << "   [+]vx:"<< absolute_vlty._x<< "  _obs_theta:" <<obs_3d._obs_theta*180/PI<<std::endl;
                    obs_3d.ori_rms = 7.00f;
                    obs_3d.oriRate_rms = 21.00f;
                }else{
                    // std::cout << "\n" << std::endl;
                    // std::cout << "small" << std::endl;
                    obs_3d._obs_theta = 0.0;
                    obs_3d.ori_rms = 100.00f;
                    obs_3d.oriRate_rms = 100.00f;    
                }
                obs_3d._course_angle_rad = course_angle_rad;

                obs_3d.objProbExist = msg.obs[i].objProbExist;
                obs_3d.objDynProp = msg.obs[i].objDynProp;
                obs_3d.objMeasState = msg.obs[i].objMeasState;
                obs_3d.obj_amp = msg.obs[i].obj_amp;

                Data_Projection(obs_3d);

                if((obs_3d.objMeasState != 2) || (obs_3d.objProbExist<5) || (obs_3d.obj_amp < 0)){
                    continue;
                }
                // std::cout << "objMeasState:" << obs_3d.objMeasState << std::endl;
                obs_3d.objClass = msg.obs[i].objClass;
                obs_3d._acceleration.x = msg.obs[i].accx;
                obs_3d._acceleration.y = msg.obs[i].accy;
                //obs_3d.rcs = msg.obstacles[i].rcs;
                obs_3d_obstacles.push_back(obs_3d);
                //ROS_INFO("msg.obs[i].id:%d,msg.obs[i].x:%f,msg.obs[i].y:%f,msg.obs[i].z:%f",msg.obs[i].id,msg.obs[i].x,msg.obs[i].y,obs_3d._obs_position._z);
            }
        }

        //  fusion_raw_data_visualization(obs_3d_obstacles);    
        /*
            _obstacle_ids = current_ids;
            radar_his_obstacles = obs_3d_obstacles;
        */
        if (obs_3d_obstacles.size() != 0){
            obs_3d_frame.set_obstacles(obs_3d_obstacles);
            _Engine.process_radar_frame(obs_3d_frame);
            publish_final_fusion();
            fusion_visualization();
        }
            // _visualizer.visualize_radar_frame(obs_3d_frame);
    }
    
    static void obs_callback_local_rear_radar_online(const ros_fusion::ivsensorlrrobj &msg) {

        // std::cout << "Now we enter the rear radar callback" << std::endl;

        rear_radar_time = msg.header.stamp.toSec();
        rear_now_time = rear_radar_time;
        judge_control_mode(1);
        uint64_t unix_time_usec = static_cast<uint64_t>(msg.header.stamp.toSec() * 1e6);
        uint64_t gps_time_usec = unix2gps(unix_time_usec);
        uint64_t sec =  gps_time_usec / 1000000;
        uint64_t nsec =  (gps_time_usec % 1000000) * 1000;

        //Eigen::Matrix4d radar_pose = Eigen::Matrix4d::Identity();
        // now, the transform is from the radar point to cloud coordinate
        Eigen::Matrix4d radar_pose = transform.matrix();            

        tsinghua::dias::fusion::OBSFHeader header;
        header._seq = msg.header.seq;
        // header._stamp._sec = msg.header.stamp.sec;
        header._stamp._sec = sec;
        // header._stamp._nsec = msg.header.stamp.nsec;
        header._stamp._nsec = nsec;
        header._frame_id = msg.header.frame_id;           

        tsinghua::dias::fusion::OBSFObs3dFrame obs_3d_frame;
        obs_3d_frame.set_header(header);        

        std::vector<tsinghua::dias::fusion::OBSFObs3d> obs_3d_obstacles;
        std::set<int> current_ids;

        

        for (size_t i = 0; i < msg.obs.size(); i++)
        {
            if(msg.obs[i].x>0){

                tsinghua::dias::fusion::OBSFObs3d obs_3d;
                obs_3d._header._seq = msg.header.seq;
                obs_3d._header._stamp._sec = sec;
                obs_3d._header._stamp._nsec = nsec;
                obs_3d._header._frame_id = msg.header.frame_id;
                obs_3d._sensor_header =  msg.header;
                obs_3d._obs_id = msg.obs[i].id;
                obs_3d.flag_repeat = 0;

                 
                obs_3d.rangex_rms = msg.obs[i].rangex_rms;                
                obs_3d.speedx_rms = msg.obs[i].speedx_rms;                
                obs_3d.accx_rms = msg.obs[i].accx_rms;
                obs_3d.rangey_rms = msg.obs[i].rangey_rms;
                obs_3d.speedy_rms = msg.obs[i].speedy_rms;
                obs_3d.accy_rms = msg.obs[i].accy_rms;
                obs_3d.rangez_rms = 7.00f; 
                obs_3d.speedz_rms = 100.00f;
                obs_3d._sensor_type = 3;
                obs_3d._object_type = 0;
                
                float _obs_x=msg.obs[i].x*cosf(LRR_B_ANGLE)+msg.obs[i].y*sinf(LRR_B_ANGLE);
                float _obs_y=msg.obs[i].y*cosf(LRR_B_ANGLE)-msg.obs[i].x*sinf(LRR_B_ANGLE);
                obs_3d._obs_position._x=_obs_x+LRR_B_pos_x;
                obs_3d._obs_position._y=_obs_y+LRR_B_pos_y;
                obs_3d._obs_position._z=1;
                obs_3d._velocity._x=msg.obs[i].speedx*cosf(LRR_B_ANGLE)+msg.obs[i].speedy*sinf(LRR_B_ANGLE);
                obs_3d._velocity._y=msg.obs[i].speedy*cosf(LRR_B_ANGLE)-msg.obs[i].speedx*sinf(LRR_B_ANGLE);
                obs_3d._velocity._z=0.0;
                obs_3d._self_velocity._x = glb_twist_linear_x*cosf(course_angle_rad)+glb_twist_linear_y*sinf(course_angle_rad);
                obs_3d._self_velocity._y = glb_twist_linear_y*cosf(course_angle_rad)-glb_twist_linear_x*sinf(course_angle_rad);
                obs_3d._self_velocity._z = glb_twist_linear_z;
                if( sqrt(pow(obs_3d._self_velocity._x,2)+pow(obs_3d._self_velocity._y,2))>0.3 ){
                    obs_3d._obs_theta = atan2(obs_3d._self_velocity._y, obs_3d._self_velocity._x);
                    obs_3d.ori_rms = 7.00f;
                    obs_3d.oriRate_rms = 21.00f;
                }else{
                    obs_3d._obs_theta = 0.0;
                    obs_3d.ori_rms = 100.00f;
                    obs_3d.oriRate_rms = 100.00f;    
                }
                    
                obs_3d._course_angle_rad = course_angle_rad;

                obs_3d.objProbExist = msg.obs[i].objProbExist;
                obs_3d.objDynProp = msg.obs[i].objDynProp;
                obs_3d.objMeasState = msg.obs[i].objMeasState;
                obs_3d.obj_amp = msg.obs[i].obj_amp;

                Data_Projection(obs_3d);

                if((obs_3d.objMeasState != 2) || (obs_3d.objProbExist<5) || (obs_3d.obj_amp < 0)){
                    continue;
                }
                // std::cout << "objMeasState:" << obs_3d.objMeasState << std::endl;
                obs_3d.objClass = msg.obs[i].objClass;
                //obs_3d.rcs = msg.obstacles[i].rcs;
                obs_3d._acceleration.x = msg.obs[i].accx;
                obs_3d._acceleration.y = msg.obs[i].accy;
                obs_3d_obstacles.push_back(obs_3d);
               
                
                //ROS_INFO("msg.obs[i].id:%d,msg.obs[i].x:%f,msg.obs[i].y:%f,msg.obs[i].z:%f",msg.obs[i].id,msg.obs[i].x,msg.obs[i].y,obs_3d._obs_position._z);
            }
        }

        //  fusion_raw_data_visualization(obs_3d_obstacles);    
        /*
            _obstacle_ids = current_ids;
            radar_his_obstacles = obs_3d_obstacles;
        */
        if (obs_3d_obstacles.size() != 0){
            obs_3d_frame.set_obstacles(obs_3d_obstacles);
            _Engine.process_radar_frame(obs_3d_frame);
            publish_final_fusion();
            fusion_visualization();
        }
            // _visualizer.visualize_radar_frame(obs_3d_frame);
    }


    static void obs_callback_local_right_front_radar_online(const ros_fusion::ivsensorsrrobj &msg){

        // std::cout << "Now we enter the right front radar callback" << std::endl;

        srr_radar_time = msg.header.stamp.toSec();
        now_time = srr_radar_time;
        judge_control_mode(0);

        uint64_t unix_time_usec = static_cast<uint64_t>(msg.header.stamp.toSec() * 1e6);
        uint64_t gps_time_usec = unix2gps(unix_time_usec);
        uint64_t sec =  gps_time_usec / 1000000;
        uint64_t nsec =  (gps_time_usec % 1000000) * 1000;

        //Eigen::Matrix4d radar_pose = Eigen::Matrix4d::Identity();
        // now, the transform is from the radar point to cloud coordinate
        Eigen::Matrix4d radar_pose = transform.matrix();    
        

        tsinghua::dias::fusion::OBSFHeader header;
        header._seq = msg.header.seq;
        // header._stamp._sec = msg.header.stamp.sec;
        header._stamp._sec = sec;
        // header._stamp._nsec = msg.header.stamp.nsec;
        header._stamp._nsec = nsec;
        header._frame_id = msg.header.frame_id;           

        tsinghua::dias::fusion::OBSFObs3dFrame obs_3d_frame;
        obs_3d_frame.set_header(header);        

        std::vector<tsinghua::dias::fusion::OBSFObs3d> obs_3d_obstacles;
        std::set<int> current_ids;

        for (size_t i = 0; i < msg.obs.size(); i++)
        {
            if(!(msg.obs[i].x==0 && msg.obs[i].y==0)){

                tsinghua::dias::fusion::OBSFObs3d obs_3d;
                obs_3d._header._seq = msg.header.seq;
                obs_3d._header._stamp._sec = sec;
                obs_3d._header._stamp._nsec = nsec;
                obs_3d._header._frame_id = msg.header.frame_id;
                obs_3d._sensor_header =  msg.header;
                obs_3d._obs_id = msg.obs[i].id;
                obs_3d.flag_repeat = 0;
                 

                obs_3d._sensor_type = 1;
                obs_3d._object_type = 0;
                Eigen::Matrix<double, 4, 1> location_r;
                Eigen::Matrix<double, 4, 1> location_w;
                double local_pi = tsinghua::dias::fusion::PI;
                float _obs_x=msg.obs[i].x*cosf(SRR_FR_ANGLE)+msg.obs[i].y*sinf(SRR_FR_ANGLE);
                float _obs_y=msg.obs[i].y*cosf(SRR_FR_ANGLE)-msg.obs[i].x*sinf(SRR_FR_ANGLE);
                obs_3d._obs_position._x=_obs_x+SRR_FR_pos_x;
                obs_3d._obs_position._y=_obs_y+SRR_FR_pos_y;
                obs_3d._obs_position._z=1;
                obs_3d._velocity._x=msg.obs[i].speedx*cosf(SRR_FR_ANGLE)+msg.obs[i].speedy*sinf(SRR_FR_ANGLE);
                obs_3d._velocity._y=msg.obs[i].speedy*cosf(SRR_FR_ANGLE)-msg.obs[i].speedx*sinf(SRR_FR_ANGLE);
                obs_3d._velocity._z = 0.0;
                obs_3d._obs_theta = 0.0;
    
                float track_slope = (obs_3d._obs_position._x - SRR_FR_pos_x) / (obs_3d._obs_position._y);
                if ((track_slope > lidar_right_slope) && (obs_3d._obs_position._y < 0)){
                    obs_3d.rangey_rms = 14.00f;
                    obs_3d.rangex_rms = 14.00f;
                    obs_3d.speedy_rms = 14.00f;
                    obs_3d.speedx_rms = 14.00f;
                }
                else{
                    obs_3d.rangey_rms = 100.00f;
                    obs_3d.rangex_rms = 100.00f;
                    obs_3d.speedy_rms = 100.00f;
                    obs_3d.speedx_rms = 100.00f;
                }
                obs_3d.rangez_rms = 7.00f; 
                obs_3d.speedz_rms = 100.00f;
                obs_3d.ori_rms = 21.00f;
                obs_3d.oriRate_rms = 21.00f;
                obs_3d.accx_rms = 14.00f;
                obs_3d.accy_rms = 14.00f;

                obs_3d._self_velocity._x = glb_twist_linear_x*cosf(course_angle_rad)+glb_twist_linear_y*sinf(course_angle_rad);
                obs_3d._self_velocity._y = glb_twist_linear_y*cosf(course_angle_rad)-glb_twist_linear_x*sinf(course_angle_rad);
                obs_3d._self_velocity._z = glb_twist_linear_z;
                obs_3d._course_angle_rad = course_angle_rad;
                obs_3d._acceleration.x = 0;
                obs_3d._acceleration.y = 0;
                Data_Projection(obs_3d);

                if (f_control_mode == -1)
                {
                    obs_3d_obstacles.push_back(obs_3d);
                }else{
                    if ((track_slope > lidar_right_slope) && (obs_3d._obs_position._y < 0)){
                        obs_3d_obstacles.push_back(obs_3d);
                    }
                }             
                
                //ROS_INFO("msg.obs[i].id:%d,msg.obs[i].x:%f,msg.obs[i].y:%f,msg.obs[i].z:%f",msg.obs[i].id,msg.obs[i].x,msg.obs[i].y,obs_3d._obs_position._z);
            }
        }

            
        /*
            _obstacle_ids = current_ids;
            radar_his_obstacles = obs_3d_obstacles;
        */
        if(obs_3d_obstacles.size()!=0){
            obs_3d_frame.set_obstacles(obs_3d_obstacles);
            _Engine.process_radar_frame(obs_3d_frame);
            publish_final_fusion();
            fusion_visualization();
        }
            // _visualizer.visualize_radar_frame(obs_3d_frame);

    }

    static void debug_visual(std::vector<tsinghua::dias::fusion::OBSFObs3d> obs_array){
        visualization_msgs::MarkerArray array ;
        visualization_msgs::Marker marker;
        marker.header.frame_id ="fusionData"; 
        marker.header.stamp = ros::Time::now();

        if (obs_array.size()!=0){
            for (int i=0 ; i<obs_array.size(); i++){
                marker.id = obs_array[i]._obs_id;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = obs_array[i]._obs_position._x ;
                marker.pose.position.y = obs_array[i]._obs_position._y ;
                marker.pose.position.z = 2;

                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.8;
                marker.scale.y = 0.8;
                marker.scale.z = 0.8;
                marker.color.a = 1.0; 
                marker.color.r = 0;
                marker.color.g = 1.0;
                marker.color.b = 1.0; 
                marker.lifetime = ros::Duration(0.15);
                array.markers.push_back(marker);
            }

        }
        radar_raw_pub.publish(array);

    }


    static void obs_callback_local_right_rear_radar_online(const ros_fusion::ivsensorsrrobj &msg){

        // std::cout << "Now we enter the right rear radar callback" << std::endl;

        rear_srr_time = msg.header.stamp.toSec();
        rear_now_time = rear_srr_time;
        
        judge_control_mode(1);

        uint64_t unix_time_usec = static_cast<uint64_t>(msg.header.stamp.toSec() * 1e6);
        uint64_t gps_time_usec = unix2gps(unix_time_usec);
        uint64_t sec =  gps_time_usec / 1000000;
        uint64_t nsec =  (gps_time_usec % 1000000) * 1000;

        //Eigen::Matrix4d radar_pose = Eigen::Matrix4d::Identity();
        // now, the transform is from the radar point to cloud coordinate
        Eigen::Matrix4d radar_pose = transform.matrix();    
        

        tsinghua::dias::fusion::OBSFHeader header;
        header._seq = msg.header.seq;
        // header._stamp._sec = msg.header.stamp.sec;
        header._stamp._sec = sec;
        // header._stamp._nsec = msg.header.stamp.nsec;
        header._stamp._nsec = nsec;
        header._frame_id = msg.header.frame_id;           

        tsinghua::dias::fusion::OBSFObs3dFrame obs_3d_frame;
        obs_3d_frame.set_header(header);        

        std::vector<tsinghua::dias::fusion::OBSFObs3d> obs_3d_obstacles;
        std::set<int> current_ids;

        for (size_t i = 0; i < msg.obs.size(); i++)
        {
            if(!(msg.obs[i].x==0 && msg.obs[i].y==0)){

                tsinghua::dias::fusion::OBSFObs3d obs_3d;
                obs_3d._header._seq = msg.header.seq;
                obs_3d._header._stamp._sec = sec;
                obs_3d._header._stamp._nsec = nsec;
                obs_3d._header._frame_id = msg.header.frame_id;
                obs_3d._sensor_header =  msg.header;
                obs_3d._obs_id = msg.obs[i].id;
                obs_3d.flag_repeat = 0;
                 

                obs_3d._sensor_type = 4;
                obs_3d._object_type = 0;
                Eigen::Matrix<double, 4, 1> location_r;
                Eigen::Matrix<double, 4, 1> location_w;
                double local_pi = tsinghua::dias::fusion::PI;
                float _obs_x=msg.obs[i].x*cosf(SRR_BR_ANGLE)+msg.obs[i].y*sinf(SRR_BR_ANGLE);
                float _obs_y=msg.obs[i].y*cosf(SRR_BR_ANGLE)-msg.obs[i].x*sinf(SRR_BR_ANGLE);
                obs_3d._obs_position._x=_obs_x+SRR_BR_pos_x;
                obs_3d._obs_position._y=_obs_y+SRR_BR_pos_y;
                obs_3d._obs_position._z=1;
                obs_3d._velocity._x=msg.obs[i].speedx*cosf(SRR_BR_ANGLE)+msg.obs[i].speedy*sinf(SRR_BR_ANGLE);
                obs_3d._velocity._y=msg.obs[i].speedy*cosf(SRR_BR_ANGLE)-msg.obs[i].speedx*sinf(SRR_BR_ANGLE);
                obs_3d._velocity._z = 0.0;
                obs_3d._obs_theta = 0.0;
                float track_slope = (obs_3d._obs_position._x - SRR_BR_pos_x) / (obs_3d._obs_position._y);
                if ((track_slope < B_lidar_right_slope) && (obs_3d._obs_position._y < 0)){
                    obs_3d.rangey_rms = 14.00f;
                    obs_3d.rangex_rms = 14.00f;
                    obs_3d.speedy_rms = 14.00f;
                    obs_3d.speedx_rms = 14.00f;
                }
                else{
                    obs_3d.rangey_rms = 100.00f;
                    obs_3d.rangex_rms = 100.00f;
                    obs_3d.speedy_rms = 100.00f;
                    obs_3d.speedx_rms = 100.00f;
                }
                obs_3d.rangez_rms = 7.00f; 
                obs_3d.speedz_rms = 100.00f;
                obs_3d.ori_rms = 21.00f;
                obs_3d.oriRate_rms = 21.00f;
                obs_3d.accx_rms = 14.00f;
                obs_3d.accy_rms = 14.00f;

                obs_3d._self_velocity._x = glb_twist_linear_x*cosf(course_angle_rad)+glb_twist_linear_y*sinf(course_angle_rad);
                obs_3d._self_velocity._y = glb_twist_linear_y*cosf(course_angle_rad)-glb_twist_linear_x*sinf(course_angle_rad);
                obs_3d._self_velocity._z = glb_twist_linear_z;
                obs_3d._course_angle_rad = course_angle_rad;
                obs_3d._acceleration.x = 0;
                obs_3d._acceleration.y = 0;
                Data_Projection(obs_3d);

                if (b_control_mode == -1)
                {
                    obs_3d_obstacles.push_back(obs_3d);
                }
                else{
                    if ((track_slope < B_lidar_right_slope) && (obs_3d._obs_position._y < 0)){
                        obs_3d_obstacles.push_back(obs_3d);
                    }
                }             
                
                //ROS_INFO("msg.obs[i].id:%d,msg.obs[i].x:%f,msg.obs[i].y:%f,msg.obs[i].z:%f",msg.obs[i].id,msg.obs[i].x,msg.obs[i].y,obs_3d._obs_position._z);
            }
        }

            
        /*
            _obstacle_ids = current_ids;
            radar_his_obstacles = obs_3d_obstacles;
        */
        if(obs_3d_obstacles.size()!=0){
            obs_3d_frame.set_obstacles(obs_3d_obstacles);
            debug_visual(obs_3d_obstacles);
            _Engine.process_radar_frame(obs_3d_frame);
            publish_final_fusion();
            fusion_visualization();
        }
            // _visualizer.visualize_radar_frame(obs_3d_frame);

    }


    static void obs_callback_local_left_front_radar_online(const ros_fusion::ivsensorsrrobj &msg){
        
        // std::cout << "Now we enter the left front radar callback" << std::endl;

        srr_radar_time = msg.header.stamp.toSec();
        now_time = srr_radar_time;
        judge_control_mode(0);

        uint64_t unix_time_usec = static_cast<uint64_t>(msg.header.stamp.toSec() * 1e6);
        uint64_t gps_time_usec = unix2gps(unix_time_usec);
        uint64_t sec =  gps_time_usec / 1000000;
        uint64_t nsec =  (gps_time_usec % 1000000) * 1000;

        //Eigen::Matrix4d radar_pose = Eigen::Matrix4d::Identity();
        // now, the transform is from the radar point to cloud coordinate
        Eigen::Matrix4d radar_pose = transform.matrix();    
        

        tsinghua::dias::fusion::OBSFHeader header;
        header._seq = msg.header.seq;
        // header._stamp._sec = msg.header.stamp.sec;
        header._stamp._sec = sec;
        // header._stamp._nsec = msg.header.stamp.nsec;
        header._stamp._nsec = nsec;
        header._frame_id = msg.header.frame_id;           

        tsinghua::dias::fusion::OBSFObs3dFrame obs_3d_frame;
        obs_3d_frame.set_header(header);        

        std::vector<tsinghua::dias::fusion::OBSFObs3d> obs_3d_obstacles;
        std::set<int> current_ids;

        for (size_t i = 0; i < msg.obs.size(); i++)
        {
            if(!(msg.obs[i].x==0 && msg.obs[i].y==0)){

                tsinghua::dias::fusion::OBSFObs3d obs_3d;
                obs_3d._header._seq = msg.header.seq;
                obs_3d._header._stamp._sec = sec;
                obs_3d._header._stamp._nsec = nsec;
                obs_3d._header._frame_id = msg.header.frame_id;
                obs_3d._sensor_header =  msg.header;
                obs_3d._obs_id = msg.obs[i].id;
                obs_3d.flag_repeat = 0;
                 
                
                obs_3d._sensor_type = 2;
                obs_3d._object_type = 0;
                Eigen::Matrix<double, 4, 1> location_r;
                Eigen::Matrix<double, 4, 1> location_w;
                double local_pi = tsinghua::dias::fusion::PI;
                float _obs_x=msg.obs[i].x*cosf(SRR_FL_ANGLE)+msg.obs[i].y*sinf(SRR_FL_ANGLE);
                float _obs_y=msg.obs[i].y*cosf(SRR_FL_ANGLE)-msg.obs[i].x*sinf(SRR_FL_ANGLE);
                obs_3d._obs_position._x=_obs_x+SRR_FL_pos_x;
                obs_3d._obs_position._y=_obs_y+SRR_FL_pos_y;
                obs_3d._obs_position._z=1;
                obs_3d._velocity._x=msg.obs[i].speedx*cosf(SRR_FL_ANGLE)+msg.obs[i].speedy*sinf(SRR_FL_ANGLE);
                obs_3d._velocity._y=msg.obs[i].speedy*cosf(SRR_FL_ANGLE)-msg.obs[i].speedx*sinf(SRR_FL_ANGLE);
                obs_3d._velocity._z = 0.0;
                obs_3d._obs_theta = 0.0;

                float track_slope = (obs_3d._obs_position._x - SRR_FL_pos_x) / (obs_3d._obs_position._y);
                if ((track_slope < lidar_left_slope) && (obs_3d._obs_position._y > 0)){
                    obs_3d.rangey_rms = 14.00f;
                    obs_3d.rangex_rms = 14.00f;
                    obs_3d.speedy_rms = 14.00f;
                    obs_3d.speedx_rms = 14.00f;
                }
                else{
                    obs_3d.rangey_rms = 100.00f;
                    obs_3d.rangex_rms = 100.00f;
                    obs_3d.speedy_rms = 100.00f;
                    obs_3d.speedx_rms = 100.00f;
                }
                obs_3d.rangez_rms = 7.00f; 
                obs_3d.speedz_rms = 100.00f;
                obs_3d.ori_rms = 21.00f;
                obs_3d.oriRate_rms = 21.00f;
                obs_3d.accx_rms = 14.00f;
                obs_3d.accy_rms = 14.00f;


             
                //obs_3d.rcs = msg.obstacles[i].rcs;
                obs_3d._self_velocity._x = glb_twist_linear_x*cosf(course_angle_rad)+glb_twist_linear_y*sinf(course_angle_rad);
                obs_3d._self_velocity._y = glb_twist_linear_y*cosf(course_angle_rad)-glb_twist_linear_x*sinf(course_angle_rad);
                obs_3d._self_velocity._z = glb_twist_linear_z;
                obs_3d._course_angle_rad = course_angle_rad;

                Data_Projection(obs_3d);
                obs_3d._acceleration.x = 0;
                obs_3d._acceleration.y = 0;
                
                if (f_control_mode == -1){
                    obs_3d_obstacles.push_back(obs_3d);
                }
                else{
                    if ((track_slope < lidar_left_slope) && (obs_3d._obs_position._y > 0))
                    {
                        obs_3d_obstacles.push_back(obs_3d);
                    }
                }

                //ROS_INFO("msg.obs[i].id:%d,msg.obs[i].x:%f,msg.obs[i].y:%f,msg.obs[i].z:%f",msg.obs[i].id,msg.obs[i].x,msg.obs[i].y,obs_3d._obs_position._z);
            }
        }
        
            
        /*
            _obstacle_ids = current_ids;
            radar_his_obstacles = obs_3d_obstacles;
        */
        if(obs_3d_obstacles.size()!=0){
        obs_3d_frame.set_obstacles(obs_3d_obstacles);
        _Engine.process_radar_frame(obs_3d_frame);
        publish_final_fusion();
        fusion_visualization();
        }
            // _visualizer.visualize_radar_frame(obs_3d_frame);
    }

    static void obs_callback_local_left_rear_radar_online(const ros_fusion::ivsensorsrrobj &msg){
        
        // std::cout << "Now we enter the left rear radar callback" << std::endl;

        rear_srr_time = msg.header.stamp.toSec();
        rear_now_time = rear_srr_time;
        judge_control_mode(1);

        uint64_t unix_time_usec = static_cast<uint64_t>(msg.header.stamp.toSec() * 1e6);
        uint64_t gps_time_usec = unix2gps(unix_time_usec);
        uint64_t sec =  gps_time_usec / 1000000;
        uint64_t nsec =  (gps_time_usec % 1000000) * 1000;

        //Eigen::Matrix4d radar_pose = Eigen::Matrix4d::Identity();
        // now, the transform is from the radar point to cloud coordinate
        Eigen::Matrix4d radar_pose = transform.matrix();    
        

        tsinghua::dias::fusion::OBSFHeader header;
        header._seq = msg.header.seq;
        // header._stamp._sec = msg.header.stamp.sec;
        header._stamp._sec = sec;
        // header._stamp._nsec = msg.header.stamp.nsec;
        header._stamp._nsec = nsec;
        header._frame_id = msg.header.frame_id;           

        tsinghua::dias::fusion::OBSFObs3dFrame obs_3d_frame;
        obs_3d_frame.set_header(header);        

        std::vector<tsinghua::dias::fusion::OBSFObs3d> obs_3d_obstacles;
        std::set<int> current_ids;

        for (size_t i = 0; i < msg.obs.size(); i++)
        {
            if(!(msg.obs[i].x==0 && msg.obs[i].y==0)){

                tsinghua::dias::fusion::OBSFObs3d obs_3d;
                obs_3d._header._seq = msg.header.seq;
                obs_3d._header._stamp._sec = sec;
                obs_3d._header._stamp._nsec = nsec;
                obs_3d._header._frame_id = msg.header.frame_id;
                obs_3d._sensor_header =  msg.header;
                obs_3d._obs_id = msg.obs[i].id;
                obs_3d.flag_repeat = 0;
                 
                
                obs_3d._sensor_type = 5;
                obs_3d._object_type = 0;
                Eigen::Matrix<double, 4, 1> location_r;
                Eigen::Matrix<double, 4, 1> location_w;
                double local_pi = tsinghua::dias::fusion::PI;
                float _obs_x=msg.obs[i].x*cosf(SRR_BL_ANGLE)+msg.obs[i].y*sinf(SRR_BL_ANGLE);
                float _obs_y=msg.obs[i].y*cosf(SRR_BL_ANGLE)-msg.obs[i].x*sinf(SRR_BL_ANGLE);
                obs_3d._obs_position._x=_obs_x+SRR_BL_pos_x;
                obs_3d._obs_position._y=_obs_y+SRR_BL_pos_y;
                obs_3d._obs_position._z=1;
                obs_3d._velocity._x=msg.obs[i].speedx*cosf(SRR_BL_ANGLE)+msg.obs[i].speedy*sinf(SRR_BL_ANGLE);
                obs_3d._velocity._y=msg.obs[i].speedy*cosf(SRR_BL_ANGLE)-msg.obs[i].speedx*sinf(SRR_BL_ANGLE);
                obs_3d._velocity._z = 0.0;
                obs_3d._obs_theta = 0.0;

                float track_slope = (obs_3d._obs_position._x - SRR_BL_pos_x) / (obs_3d._obs_position._y);
                if ((track_slope > B_lidar_left_slope) && (obs_3d._obs_position._y > 0)){
                    obs_3d.rangey_rms = 14.00f;
                    obs_3d.rangex_rms = 14.00f;
                    obs_3d.speedy_rms = 14.00f;
                    obs_3d.speedx_rms = 14.00f;
                }
                else{
                    obs_3d.rangey_rms = 100.00f;
                    obs_3d.rangex_rms = 100.00f;
                    obs_3d.speedy_rms = 100.00f;
                    obs_3d.speedx_rms = 100.00f;
                }
                obs_3d.rangez_rms = 7.00f; 
                obs_3d.speedz_rms = 100.00f;
                obs_3d.ori_rms = 21.00f;
                obs_3d.oriRate_rms = 21.00f;
                obs_3d.accx_rms = 14.00f;
                obs_3d.accy_rms = 14.00f;



                //obs_3d.rcs = msg.obstacles[i].rcs;
                obs_3d._self_velocity._x = glb_twist_linear_x*cosf(course_angle_rad)+glb_twist_linear_y*sinf(course_angle_rad);
                obs_3d._self_velocity._y = glb_twist_linear_y*cosf(course_angle_rad)-glb_twist_linear_x*sinf(course_angle_rad);
                obs_3d._self_velocity._z = glb_twist_linear_z;
                obs_3d._course_angle_rad = course_angle_rad;
                obs_3d._acceleration.x = 0;
                obs_3d._acceleration.y = 0;

                Data_Projection(obs_3d);
                
                if (b_control_mode == -1){
                    obs_3d_obstacles.push_back(obs_3d);
                }
                else{
                    if ((track_slope > B_lidar_left_slope) && (obs_3d._obs_position._y > 0))
                    {
                        obs_3d_obstacles.push_back(obs_3d);
                    }
                }
            }
        }
        
            
        /*
            _obstacle_ids = current_ids;
            radar_his_obstacles = obs_3d_obstacles;
        */
        if(obs_3d_obstacles.size()!=0){
        obs_3d_frame.set_obstacles(obs_3d_obstacles);
        // debug_visual(obs_3d_obstacles);
        _Engine.process_radar_frame(obs_3d_frame);
        publish_final_fusion();
        fusion_visualization();
        }
            // _visualizer.visualize_radar_frame(obs_3d_frame);
    }

    static void obs_callback_gps_vel_online(const geometry_msgs::TwistWithCovarianceStamped& msg) 
    {
        // std::cout << "Now we enter the gps vel callback" << std::endl;
        glb_twist_linear_x = msg.twist.twist.linear.x;
        glb_twist_linear_y = msg.twist.twist.linear.y;
        glb_twist_linear_z = msg.twist.twist.linear.z;
    }

    static void obs_callback_imu_data_online(const sensor_msgs::Imu& msg) 
    {
        // std::cout << "Now we enter the imu data callback" << std::endl;
        // q.w() = msg.orientation.w;
        // q.x() = msg.orientation.x;
        // q.y()= msg.orientation.y;
        // q.z() = msg.orientation.z;
        float yaw_calibration = 1;
        imu_roll, imu_pitch, imu_yaw = QuaternionToEulerAngles(msg.orientation);
        // ROS_INFO("pitch: %f",pitch*180/3.1415926);
        // ROS_INFO("roll : %f",roll*180/3.1415926);
        // ROS_INFO("yaw  : %f", imu_yaw*180/3.1415926);
        // ROS_INFO("yaw1: %f",atan2f(2*(w*z+x*y), w*w+x*x-y*y-z*z)*180/3.1415926);
/*niu jin */
        course_angle_rad = PI/2 + yaw_calibration*imu_yaw;
        float ax = msg.linear_acceleration.x;
        // float ay = msg.linear_acceleration.y;
        float ay = msg.linear_acceleration.y;
        float az = msg.linear_acceleration.z;
        // std::cout << "rad... " << course_angle_rad*180/3.1415926 <<"     ax....  " << ax << "      ay...... " << ay <<std::endl;
        float acc_angle = imu_yaw;//*yaw_calibration;
        imu_acceleration_x = ax ;//*cosf(acc_angle)+ay*sinf(acc_angle);
        imu_acceleration_y = ay ;//*cosf(acc_angle)-ax*sinf(acc_angle);
        // std::cout << "ax: "<<ax <<"ay: "<<ay << "ax_vehicle: "<<imu_acceleration_x<< std::endl ;
        // std::cout << "vx: " << glb_twist_linear_x*cosf(course_angle_rad)+glb_twist_linear_y*sinf(course_angle_rad) 
        //         << "    vy: " <<  glb_twist_linear_y*cosf(course_angle_rad)-glb_twist_linear_x*sinf(course_angle_rad) 
        //         << "    ax: " << imu_acceleration_x \
        //         << "    ay: " << imu_acceleration_y << std::endl;
 /*zhu xian*/
        // course_angle_rad = PI/2 - imu_yaw;
        // float ax = msg.linear_acceleration.x;
        // float ay = msg.linear_acceleration.y;
        // float az = msg.linear_acceleration.z;
        // // std::cout << "imu_yaw:  "  << imu_yaw*180/3.1415926 <<"      rad:    " << course_angle_rad*180/3.1415926 << std::endl;
        // float acc_angle = PI/2 - imu_yaw;//*yaw_calibration;
        // imu_acceleration_x = ax*cosf(acc_angle)+ay*sinf(acc_angle);
        // imu_acceleration_y = ay*cosf(acc_angle)-ax*sinf(acc_angle);
        // std::cout << "vx: " << glb_twist_linear_x*cosf(course_angle_rad)+glb_twist_linear_y*sinf(course_angle_rad) 
        //     << "    vy: " <<  glb_twist_linear_y*cosf(course_angle_rad)-glb_twist_linear_x*sinf(course_angle_rad) 
        //     << "    ax: " << imu_acceleration_x \
        //     << "    ay: " << imu_acceleration_y << std::endl;
    }

    static void obs_callback_gps_fix_online(const sensor_msgs::NavSatFix::ConstPtr& msg) 
    {
        // std::cout << "Now we enter the gps fix callback" << std::endl;
        double lon, lat, lat_b, lon_b;
        float scale[2], a, f, b, e2, A, B;
        lat=msg->latitude;
        lon=msg->longitude; 
        lat_b=40.0068589313;   //路径起点需手动输入                             
        lon_b=116.327951572;
        index_gps = index_gps+1;
        if(index_gps<5){
            a=6378137.0;
            f=298.2572; 
            b=(f - 1) / f * a;
            e2=(a*a - b*b) / (a*a);
            A = a * (1 - e2) / pow((1-e2*pow(sin(lat_b/180.0*PI),2)),1.5);
            B = a * cos(lat_b/180.0*PI)/sqrt(1-e2*pow(sin(lat_b/180.0*PI),2));
            scale[0]=B*1.0/180.0*PI;
            scale[1]=A*1.0/180.0*PI;
        }
        route_x = (lon - lon_b) * scale[0];   //转换为坐标
        route_y = (lat - lat_b) * scale[1];
        // ROS_INFO("lat:%f,lon :%f",msg->latitude,msg->longitude);
        // ROS_INFO("lat:%f,lon :%f",lat_b,lon_b);
        // ROS_INFO("lat:%f,lon :%f",route_x,route_y);
    }

    // static void callback_cloud_visualization(const sensor_msgs::PointCloud2ConstPtr& msg) 
    // {
    //     std::cout << "we are now in the cloud visualization callback" << std::endl;
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr query_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //     pcl::fromROSMsg(*msg, *query_cloud);

    //     // _visualizer.remove_points();  
    //     // _visualizer.add_point_cloud(query_cloud, 200, 0, 0, 1, "new_cloud", 0);
    // }

    // static void callback_velodyne_visualization(const sensor_msgs::PointCloud2ConstPtr& msg) 
    // {
    //     std::cout << "we are now in the velodyne visualization callback" << std::endl;
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr query_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    //     pcl::fromROSMsg(*msg, *query_cloud);

    //     // _visualizer.remove_points();  
    //     // _visualizer.add_point_cloud(query_cloud, 200, 0, 0, 1, "new_cloud", 0);
    // } 

    static bool LessSort(ros_fusion::ld_Coeff a,ros_fusion::ld_Coeff b){
        return a.d > b.d;
    }    
    static void obs_callback_lane_data_online(const ros_fusion::ld_Frame& msg){
        std::vector<ros_fusion::ld_Coeff> lane_sequence = msg.lane_Coeff;
        sort(lane_sequence.begin(),lane_sequence.end(),LessSort);
        std::vector<int> AutoDrive_lane;
        int lane_sequence_number = 0;
        // for(int i=0; i<lane_sequence.size(); i++){
        //     std::cout << "lane_sequence[i].d \n" << lane_sequence[i].d << std::endl;
        // }
        for(int i=0; i<lane_sequence.size(); i++){
            if(lane_sequence[i].d<=0){
                lane_sequence_number = i;
                break;
            }   
        }
        for(int i=1; i<lane_sequence.size(); i++){
            AutoDrive_lane.push_back(lane_sequence_number-i);//get the current lane which the driving vehicle is in. 
            // std::cout << "AutoDrive_lane \n" << lane_sequence_number-i << std::endl;
        }
        double lane_time_stamp = ros::Time::now().toSec();
        _Engine.set_lane_detection_data(lane_sequence, AutoDrive_lane, lane_time_stamp);

        // publish lane
        std::vector<int> lane_points;
        for(int j=0; j<100; j+=1){ lane_points.push_back(j-20); }
        visualization_msgs::Marker lane_strip;
        lane_strip.header.frame_id = "fusionData";
        lane_strip.header.stamp = ros::Time::now();
        lane_strip.ns = "lane";
        lane_strip.lifetime=ros::Duration(1);
        lane_strip.action = visualization_msgs::Marker::ADD;//will refresh when new data goes in, not sure works or not
        lane_strip.pose.position.z = -2.5;
        lane_strip.pose.orientation.w = 1.0;
        lane_strip.type = visualization_msgs::Marker::LINE_LIST;//use line_strip represent line
        lane_strip.scale.x = 0.12;
        lane_strip.color.r = 1.0;
        lane_strip.color.g = 1.0;
        lane_strip.color.b = 1.0;//white 
        lane_strip.color.a = 1.0;
        //int laneDetected = 3;
        for (int j =0;j<lane_sequence.size();j++)
        {
            lane_strip.id =lane_sequence[0].id;//from lane detection
            // lane defines here i represents points number
            for (int i = 0; i < lane_points.size(); ++i)
            {
                float x= lane_points[i];
                float y= lane_sequence[j].a*pow(x,3)+lane_sequence[j].b*pow(x,2)
                            +lane_sequence[j].c*x + lane_sequence[j].d;                
                geometry_msgs::Point p;
                p.x = x;
                p.y = y;
                p.z = 0;
                lane_strip.points.push_back(p);
                p.x = x+2;
                p.y = y;
                p.z = 0;
                lane_strip.points.push_back(p);
            }
            lane_pub.publish(lane_strip);
        }

    } 

    static void fusion_raw_data_visualization(std::vector<tsinghua::dias::fusion::OBSFObs3d> obs_3d_lidar)
    {
        visualization_msgs::MarkerArray array,array_text,array_camera,raw_camera;
        visualization_msgs::Marker marker;
        
    // ROS_INFO("VEC SIZE:%ld",array.markers.size());
        if(obs_3d_lidar.size() !=0){
            for (int i = 0; i < obs_3d_lidar.size(); i++){
                if(obs_3d_lidar[i]._sensor_type != 7){
                    marker.header.frame_id ="fusionData";    //base_link
                    marker.header.stamp = ros::Time::now();
                    marker.ns = "my_namespace";

                    marker.id = i;//srrRadarProcData[i].id;
                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position.x = obs_3d_lidar[i]._obs_position._x;
                    marker.pose.position.y = obs_3d_lidar[i]._obs_position._y;
                    marker.pose.position.z = 2;

                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = 0.8;
                    marker.scale.y = 0.8;
                    marker.scale.z = 0.8;
                    marker.color.a = 1.0; // Don't forget to set the alpha!
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0; 
                    marker.lifetime = ros::Duration(0.3); //wangwei
                    if(marker.pose.position.x !=3.7)
                    {
                        array.markers.push_back(marker);

                //ROS_INFO("VEC SIZE 2:%ld",array.markers.size());           
                        char strTmp[90];
                        sprintf(strTmp, "ID:%d,X:%0.2f,Y:%0.2f,vx:%0.2f,vy:%0.2f",
                        obs_3d_lidar[i]._obs_id,
                        obs_3d_lidar[i]._obs_position._x,
                        obs_3d_lidar[i]._obs_position._y,
                        obs_3d_lidar[i]._velocity._x,
                        obs_3d_lidar[i]._velocity._y);		
                        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                        marker.pose.position.z = 3.5;//1.5
                        marker.scale.z = 0.3;
                        marker.text = strTmp;
                        array_text.markers.push_back(marker);
                    }
                }else{
                    marker.header.frame_id ="fusionData";    //base_link
                    marker.header.stamp = ros::Time::now();
                    marker.ns = "my_namespace";

                    marker.id = i;//srrRadarProcData[i].id;
                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position.x = obs_3d_lidar[i]._obs_position._x;
                    marker.pose.position.y = obs_3d_lidar[i]._obs_position._y;
                    marker.pose.position.z = 2;

                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = 0.8;
                    marker.scale.y = 0.8;
                    marker.scale.z = 0.8;
                    marker.color.a = 1.0; // Don't forget to set the alpha!
                    marker.color.r = 0.98;
                    marker.color.g = 0.8;
                    marker.color.b = 0.8;
                
                    marker.lifetime = ros::Duration(0.8); //wangwei
                    if(marker.pose.position.x !=3.7)
                    {
                        raw_camera.markers.push_back(marker);

                //ROS_INFO("VEC SIZE 2:%ld",array.markers.size());           
                        char strTmp[90];
                        sprintf(strTmp, "ID:%d,X:%0.2f,Y:%0.2f,vx:%0.2f,vy:%0.2f",
                        obs_3d_lidar[i]._obs_id,
                        obs_3d_lidar[i]._obs_position._x,
                        obs_3d_lidar[i]._obs_position._y,
                        obs_3d_lidar[i]._velocity._x,
                        obs_3d_lidar[i]._velocity._y);		
                        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                        marker.pose.position.z = 3.5;//1.5
                        marker.scale.z = 0.3;
                        marker.text = strTmp;
                        array_camera.markers.push_back(marker);
                    }
                }
            }
        }

        visualization_msgs::MarkerArray object_boxes;
        visualization_msgs::Marker marker_boxes;

        for (int i = 0; i < obs_3d_lidar.size(); i++)
        {
            marker_boxes.header.frame_id = "fusionData";
            marker_boxes.header.stamp = ros::Time::now();
            marker_boxes.ns = "my_namespace";
            marker_boxes.id = i;
            marker_boxes.type = visualization_msgs::Marker::CUBE;
            marker_boxes.action = visualization_msgs::Marker::ADD;
            marker_boxes.color.a = 0.75;
            marker_boxes.color.r = 1;
            marker_boxes.color.g = 0;
            marker_boxes.color.b = 1;
            marker_boxes.lifetime = ros::Duration(0.3);

            marker_boxes.pose = obs_3d_lidar[i].object_box_center.pose;
            marker_boxes.scale = obs_3d_lidar[i].object_box_size;
            //std::cout << " x " << _obs_track[i].get_obs_radar()->object_box_size.x << " y " << _obs_track[i].get_obs_radar()->object_box_size.y << std::endl;
            if (marker_boxes.scale.x == 0.0)
            {
                marker_boxes.scale.x = 0.01;
            }
            if (marker_boxes.scale.y == 0.0)
            {
                marker_boxes.scale.y = 0.01;
            }
            marker_boxes.scale.z = 1;
            object_boxes.markers.push_back(marker_boxes);
        }

        visualization_msgs::MarkerArray object_velocity;
        visualization_msgs::Marker marker_velocity;
        int ii=0;
        for (int i = 0; i < obs_3d_lidar.size(); i++){
            if (sqrt(pow(obs_3d_lidar[i]._velocity._x, 2) + pow(obs_3d_lidar[i]._velocity._y, 2)) < 3)
            {
                ii=ii+1;
                marker_velocity.header.frame_id = "fusionData";
                marker_velocity.header.stamp = ros::Time::now();
                marker_velocity.ns = "my_namespace";
                marker_velocity.id = ii;
                marker_velocity.type = visualization_msgs::Marker::ARROW;
                marker_velocity.action = visualization_msgs::Marker::ADD;
                marker_velocity.scale.x = 0.4;
                marker_velocity.scale.y = 0.7;
                marker_velocity.color.a = 0.75;
                marker_velocity.color.r = 1;
                marker_velocity.color.g = 1;
                marker_velocity.color.b = 0;
                //marker_velocity.lifetime = ros::Duration(0.8);

                marker_velocity.points.resize(2);
                marker_velocity.points[0].x = obs_3d_lidar[i]._obs_position._x;
                marker_velocity.points[0].y = obs_3d_lidar[i]._obs_position._y;
                marker_velocity.points[1].x = obs_3d_lidar[i]._obs_position._x + obs_3d_lidar[i]._velocity._x;
                marker_velocity.points[1].y = obs_3d_lidar[i]._obs_position._y + obs_3d_lidar[i]._velocity._y;

                //std::cout << " x " << _obs_track[i].get_obs_radar()->object_box_size.x << " y " << _obs_track[i].get_obs_radar()->object_box_size.y << std::endl;

                object_velocity.markers.push_back(marker_velocity);
            }
        }

        visualization_msgs::MarkerArray contour_lines;
        // visualization_msgs::Marker marker_contour;
        contour_lines.markers.resize(obs_3d_lidar.size());
        for (int i = 0; i < obs_3d_lidar.size(); i++)
        {
            contour_lines.markers[i].header.frame_id = "fusionData";
            contour_lines.markers[i].header.stamp = ros::Time::now();
            contour_lines.markers[i].ns = "my_namespace";
            contour_lines.markers[i].id = i;
            contour_lines.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
            contour_lines.markers[i].action = visualization_msgs::Marker::ADD;
            contour_lines.markers[i].scale.x = 0.1;
            contour_lines.markers[i].color.a = 0.75;
            contour_lines.markers[i].color.r = 1;
            contour_lines.markers[i].color.g = 1;
            contour_lines.markers[i].color.b = 0;
            contour_lines.markers[i].lifetime = ros::Duration(0.8);

            for(int j=0;j<obs_3d_lidar[i].contour_points.size();j++){
                contour_lines.markers[i].points.push_back(obs_3d_lidar[i].contour_points[j]);
            }
            //std::cout << " x " << _obs_track[i].get_obs_radar()->object_box_size.x << " y " << _obs_track[i].get_obs_radar()->object_box_size.y << std::endl;

            //contour_lines.markers[i] = marker_contour;
        }

        // ROS_INFO("pub fusion data");
        pub_lidar_box.publish(object_boxes);
        pub_lidar_velocity.publish(object_velocity);
        pub_lidar_contour.publish(contour_lines);
        pub_lidar_markerArray.publish(array);
        pub_camera_markerArray.publish(raw_camera);
        pub_lidar_text.publish(array_text);
        pub_camera_text.publish(array_camera);
    }


    static void fusion_visualization()
    {
        visualization_msgs::MarkerArray array,array_text;
        visualization_msgs::Marker marker;
        

        std::vector<tsinghua::dias::fusion::OBSFUncertaintyTrack> _obs_track;
        _obs_track=_Engine.get_tracks();
    // ROS_INFO("VEC SIZE:%ld",array.markers.size());
        if(_obs_track.size() !=0){
            for (int i = 0; i < _obs_track.size(); i++){
                if(_obs_track[i].get_obs_radar()->flag_repeat < 2){  //flag_repeat 初始化为　0
                    continue;
                }
        //                 std::cout << "track  p:\n" << _obs_track[i].get_obs_radar()->P << std::endl;
        // std::cout << "track  xy:\n" << _obs_track[i].get_obs_radar()->_obs_position._x << "  "<<_obs_track[i].get_obs_radar()->_obs_position._y << std::endl;
        // std::cout << "track  vv:\n" << _obs_track[i].get_obs_radar()->_velocity._x <<"  " << _obs_track[i].get_obs_radar()->_velocity._y  <<std::endl;
                marker.header.frame_id ="fusionData";    //base_link
                marker.header.stamp = ros::Time::now();
                marker.ns = "my_namespace";

                marker.id = i;//srrRadarProcData[i].id;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = _obs_track[i].get_obs_radar()->_obs_position._x;
                marker.pose.position.y = _obs_track[i].get_obs_radar()->_obs_position._y;
                marker.pose.position.z = 2;

                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.6;
                marker.scale.y = 0.6;
                marker.scale.z = 0.6;
                marker.color.a = 1.0; // Don't forget to set the alpha!
                //marker.color.r = 0.0;
                //marker.color.g = 1.0;

                // if (_obs_track[i].get_obs_radar()->flag_repeat < 5)
                // {
                //     //continue;
                //     marker.color.r = 0.0;
                //     marker.color.g = _obs_track[i].get_obs_radar()->flag_repeat / 5.0;
                //     marker.color.b = 0.0;
                // }else if (_obs_track[i].get_obs_radar()->flag_repeat < 10)
                // {
                //     marker.color.r = 0.0;
                //     marker.color.g = 1.0;
                //     marker.color.b = 0.0;
                // }else{
                //     //marker.pose.position.z = 6;
                //     marker.color.r = 0.0;
                //     marker.color.g = 0.0;
                //     marker.color.b = 1.0;
                // }
                int trk_ln = _obs_track[i].get_obs_radar()->_tracks_lane;
                if( trk_ln== 11){
                    /* RGB Diag*/
                    int t = _obs_track[i].get_obs_radar()->_sensor_type;
                    if(t == 0 || t == 3){ //red
                        marker.color.r = 255.0/255;  
                        marker.color.g = 106.0/255;
                        marker.color.b = 106.0/255;
                    }else if(t == 6 || t == 8){ //blue
                        marker.color.r = 0.541;  
                        marker.color.g = 0.169;
                        marker.color.b = 0.886;
                    }else if(t == 1 || t == 4){ //yellow
                        marker.color.r = 255.0/255;
                        marker.color.g = 255.0/255;
                        marker.color.b = 0.0/255;
                    }else if(t == 2 || t == 5){ //green
                        marker.color.r = 0.0/255;
                        marker.color.g = 255.0/255;
                        marker.color.b = 0.0/255;
                    }
                    if(_obs_track[i].get_obs_radar()->flag_repeat > 2){
                        marker.color.r = 1.0;
                        marker.color.g = 1.0;
                        marker.color.b = 1.0;
                    }
                }else{
                    if(trk_ln == 9){
                        marker.color.r = 1;
                        marker.color.g = 0;
                        marker.color.b = 0;
                    }else if(trk_ln == -9){
                        marker.color.r = 0;
                        marker.color.g = 0;
                        marker.color.b = 1;
                    }else{

                        if(trk_ln == 0){
                            marker.color.r = 0;
                            marker.color.g = 1;
                            marker.color.b = 0;
                        }else if(trk_ln == 1){
                            marker.color.r = 0;
                            marker.color.g = 1;
                            marker.color.b = 1;
                        }else if(trk_ln == -1){
                            marker.color.r = 1;//yellow
                            marker.color.g = 1;
                            marker.color.b = 0;
                        }else{
                            marker.color.r = 0.5;
                            marker.color.g = 1;
                            marker.color.b = 0.5;
                        }                       
                    }
                }
                if( _obs_track[i].get_obs_radar()->_object_type == 1){
                    marker.scale.x = 1.0;
                    marker.scale.y = 1.0;
                    marker.scale.z = 1.0;
                    marker.color.a = 1.0;
                    ROS_WARN("_obs_track[%d].get_obs_radar()->_ctype_update: %d",_obs_track[i].get_obs_radar()->_obs_id,_obs_track[i].get_obs_radar()->_ctype_update);
                }
                //marker.color.b = 0.0;
                marker.lifetime = ros::Duration(0.5); //wangwei
                if(marker.pose.position.x != 3.7)//&& _obs_track[i].get_obs_radar()->flag_repeat > 1)
                {
                    array.markers.push_back(marker);
                    char strTmp[90];
                    // sprintf(strTmp, "ID:%d,ot:%d,st:%d,X:%0.2f,Y:%0.2f,fg:%d,lane:%d",
                    //         _obs_track[i].get_obs_radar()->_obs_id,
                    //         _obs_track[i].get_obs_radar()->_object_type,
                    //         _obs_track[i].get_obs_radar()->_sensor_type,
                    //         _obs_track[i].get_obs_radar()->_obs_position._x,
                    //         _obs_track[i].get_obs_radar()->_obs_position._y,
                    //         _obs_track[i].get_obs_radar()->flag_repeat,
                    //         _obs_track[i].get_obs_radar()->_tracks_lane);
                    sprintf(strTmp, "ID:%d,ot:%d,theta:%0.2f",
                            _obs_track[i].get_obs_radar()->_obs_id,
                            _obs_track[i].get_obs_radar()->_object_type,
                            _obs_track[i].get_obs_radar()->_obs_theta
                         );
                    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                    marker.pose.position.z = 3.5;
                    marker.scale.z = 0.3;
                    marker.text = strTmp;
                    array_text.markers.push_back(marker);
                }
            }
        }
    //visualization_msgs::Marker marker;
        marker.header.frame_id = "fusionData";
        marker.header.stamp = ros::Time::now();
        marker.ns = "my_namespace";
        marker.id = 1000;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.position.x = 20;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        array.markers.push_back(marker)  ;

        //show the origin　原点
        //visualization_msgs::Marker marker;
        marker.header.frame_id = "fusionData";
        marker.header.stamp = ros::Time::now();
        marker.ns = "my_namespace";
        marker.id = 1001;
        marker.type = visualization_msgs::Marker::CUBE;
        //marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        array.markers.push_back(marker);         
   
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "fusionData";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "my_namespace";
        line_strip.id = 1002;
        line_strip.action= visualization_msgs::Marker::ADD;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.pose.orientation.x = 0.0;
        line_strip.pose.orientation.y = 0.0;
        line_strip.pose.orientation.z = 0.0;
        line_strip.pose.orientation.w = 1.0;
        line_strip.scale.x = 0.3;
        line_strip.scale.y = 0.3;
        line_strip.scale.z = 0.3;
        line_strip.color.a = 1.0; // Don't forget to set the alpha!
        line_strip.color.r = 1.0;
        line_strip.color.g = 1.0;
        line_strip.color.b = 1.0;
        geometry_msgs::Point p;
        p.x = lidar_x_max_dist;
        p.y = lidar_x_max_dist/lidar_right_slope;
        p.z = 0;
        line_strip.points.push_back(p);
        p.x = lidar_x_max_dist;
        p.y = lidar_x_max_dist/lidar_left_slope;
        p.z = 0;
        line_strip.points.push_back(p);
        p.x = LRR_F_pos_x;
        p.y = LRR_F_pos_y;
        p.z = 0;
        line_strip.points.push_back(p);
        p.x = lidar_x_max_dist;
        p.y = lidar_x_max_dist/lidar_right_slope;
        p.z = 0;
        line_strip.points.push_back(p);

        geometry_msgs::Point b_p;
        b_p.x = B_lidar_x_max_dist;
        b_p.y = B_lidar_x_max_dist/B_lidar_right_slope;
        b_p.z = 0;
        line_strip.points.push_back(b_p);
        b_p.x = B_lidar_x_max_dist;
        b_p.y = B_lidar_x_max_dist/B_lidar_left_slope;
        b_p.z = 0;
        line_strip.points.push_back(b_p);
        b_p.x = LRR_B_pos_x;
        b_p.y = LRR_B_pos_y;
        b_p.z = 0;
        line_strip.points.push_back(b_p);
        b_p.x = B_lidar_x_max_dist;
        b_p.y = B_lidar_x_max_dist/B_lidar_right_slope;
        b_p.z = 0;
        line_strip.points.push_back(b_p);

        visualization_msgs::MarkerArray object_boxes;
        visualization_msgs::Marker marker_boxes;
       
        for (size_t i = 0; i < _obs_track.size(); i++)
        {
            if(_obs_track[i].get_obs_radar()->flag_repeat > 1){
                marker_boxes.header.frame_id = "fusionData";
                marker_boxes.header.stamp = ros::Time::now();
                marker_boxes.ns = "my_namespace";
                marker_boxes.id = i;
                marker_boxes.type = visualization_msgs::Marker::CUBE;
                marker_boxes.action = visualization_msgs::Marker::ADD;
                marker_boxes.color.a = 0.75;
                marker_boxes.color.r = 1;
                marker_boxes.color.g = 0;
                marker_boxes.color.b = 1;
                marker_boxes.lifetime = ros::Duration(0.5);
                geometry_msgs::Point lidar_box;
                lidar_box.x = _obs_track[i].get_obs_radar()->object_box_size.x;
                lidar_box.y = _obs_track[i].get_obs_radar()->object_box_size.y;
                lidar_box.z = _obs_track[i].get_obs_radar()->object_box_size.z;
                // if(i<3)    
                //     std::cout << "[+]sensorT:"<<_obs_track[i].get_obs_radar()->_sensor_type<< " ID:"<<_obs_track[i].get_obs_radar()->_obs_id << " x:"<<lidar_box.x<<std::endl;
                
                
                marker_boxes.pose = getEulerAnglesToQuaternion(_obs_track[i].get_obs_radar()->_object_type,\
                                                                    _obs_track[i].get_obs_radar()->_obs_position,\
                                                                    lidar_box,\
                                                                    _obs_track[i].get_obs_radar()->_obs_theta).pose;                                                                   
                // marker_boxes.pose.position.x = _obs_track[i].get_obs_radar()->_obs_position._x;
                // marker_boxes.pose.position.y = _obs_track[i].get_obs_radar()->_obs_position._y;
                // marker_boxes.pose.position.z = _obs_track[i].get_obs_radar()->_obs_position._z;
                // marker_boxes.pose.orientation = EulerAnglesToQuaternion(0,0,_obs_track[i].get_obs_radar()->_obs_theta);
                marker_boxes.scale = _obs_track[i].get_obs_radar()->object_box_size;
                //std::cout << " x " << _obs_track[i].get_obs_radar()->object_box_size.x << " y " << _obs_track[i].get_obs_radar()->object_box_size.y << std::endl;
                if (marker_boxes.scale.x == 0.0){
                    marker_boxes.scale.x = 0.01;
                }
                if (marker_boxes.scale.y == 0.0){
                    marker_boxes.scale.y = 0.01;
                }
                // marker_boxes.scale.x = 4;
                // marker_boxes.scale.y = 2;
                marker_boxes.scale.z = 1;
                object_boxes.markers.push_back(marker_boxes); 
            }
        }

        visualization_msgs::MarkerArray contour_lines;
        int jj=0;
        // visualization_msgs::Marker marker_contour;
        for (size_t i = 0; i < _obs_track.size(); i++)
        {
            if (_obs_track[i].get_obs_radar()->flag_repeat > 1)
            {
                jj=jj+1;
                contour_lines.markers.resize(jj);
                contour_lines.markers[jj - 1].header.frame_id = "fusionData";
                contour_lines.markers[jj - 1].header.stamp = ros::Time::now();
                contour_lines.markers[jj - 1].ns = "my_namespace";
                contour_lines.markers[jj - 1].id = i;
                contour_lines.markers[jj - 1].type = visualization_msgs::Marker::LINE_STRIP;
                contour_lines.markers[jj - 1].action = visualization_msgs::Marker::ADD;
                contour_lines.markers[jj - 1].scale.x = 0.1;
                contour_lines.markers[jj - 1].color.a = 0.75;
                contour_lines.markers[jj - 1].color.r = 1;
                contour_lines.markers[jj - 1].color.g = 1;
                contour_lines.markers[jj - 1].color.b = 0;
                contour_lines.markers[jj - 1].lifetime = ros::Duration(0.3);
                for (int j = 0; j < _obs_track[i].get_obs_radar()->contour_points.size(); j++){
                    contour_lines.markers[jj - 1].points.push_back(_obs_track[i].get_obs_radar()->contour_points[j]);
                }
                //std::cout << " x " << _obs_track[i].get_obs_radar()->object_box_size.x << " y " << _obs_track[i].get_obs_radar()->object_box_size.y << std::endl;

                //std::cout << "temp safe..." << std::endl;
                //contour_lines.markers[jj-1] = marker_contour;
                //MarkerArray, marker.push_back will create one single connected line strip.
                //However, for cube, sphere and other isolated markers, this won't make a difference.
            }
        }

        visualization_msgs::MarkerArray object_velocity;
        visualization_msgs::Marker marker_velocity;
        for (size_t i = 0; i < _obs_track.size(); i++)
        {
            if (_obs_track[i].get_obs_radar()->flag_repeat > 1)
            {
                marker_velocity.header.frame_id = "fusionData";
                marker_velocity.header.stamp = ros::Time::now();
                marker_velocity.ns = "my_namespace";
                marker_velocity.id = i;
                marker_velocity.type = visualization_msgs::Marker::ARROW;
                marker_velocity.action = visualization_msgs::Marker::ADD;
                marker_velocity.scale.x = 0.4;
                marker_velocity.scale.y = 0.7;
                marker_velocity.color.a = 0.75;
                marker_velocity.color.r = 1;
                marker_velocity.color.g = 0.9;
                marker_velocity.color.b = 0;
                marker_velocity.lifetime = ros::Duration(0.3);
                marker_velocity.points.resize(2);
                marker_velocity.points[0].x = _obs_track[i].get_obs_radar()->_obs_position._x;
                marker_velocity.points[0].y = _obs_track[i].get_obs_radar()->_obs_position._y;
                marker_velocity.points[1].x = _obs_track[i].get_obs_radar()->_obs_position._x
                                                + _obs_track[i].get_obs_radar()->_velocity._x
                                                + _obs_track[i].get_obs_radar()->_self_velocity._x;
                marker_velocity.points[1].y = _obs_track[i].get_obs_radar()->_obs_position._y
                                                + _obs_track[i].get_obs_radar()->_velocity._y
                                                + _obs_track[i].get_obs_radar()->_self_velocity._y;

                //std::cout << _obs_track[i].get_obs_radar()->_self_velocity._x << std::endl;
                //std::cout << " x " << _obs_track[i].get_obs_radar()->object_box_size.x << " y " << _obs_track[i].get_obs_radar()->object_box_size.y << std::endl;
                object_velocity.markers.push_back(marker_velocity);
            }           
        }
        marker_velocity.header.frame_id = "fusionData";
        marker_velocity.header.stamp = ros::Time::now();
        marker_velocity.ns = "my_namespace";
        marker_velocity.id = _obs_track.size();
        marker_velocity.type = visualization_msgs::Marker::ARROW;
        marker_velocity.action = visualization_msgs::Marker::ADD;
        marker_velocity.scale.x = 0.4;
        marker_velocity.scale.y = 0.7;
        marker_velocity.color.a = 0.75;
        marker_velocity.color.r = 1;
        marker_velocity.color.g = 0;
        marker_velocity.color.b = 0;
        marker_velocity.lifetime = ros::Duration(0.3);
        marker_velocity.points.resize(2);
        marker_velocity.points[0].x = 0;
        marker_velocity.points[0].y = 0;
        marker_velocity.points[1].x = 0 + glb_twist_linear_x * cosf(course_angle_rad) + glb_twist_linear_y * sinf(course_angle_rad);
        // marker_velocity.points[1].y = 0 + glb_twist_linear_y * cosf(course_angle_rad) - glb_twist_linear_x * sinf(course_angle_rad);
        marker_velocity.points[1].y = 0;
        object_velocity.markers.push_back(marker_velocity);


        visualization_msgs::MarkerArray MarkerArray_imu_acceleration;
        visualization_msgs::Marker marker_imu_acceleration;

        marker_imu_acceleration.header.frame_id = "fusionData";
        marker_imu_acceleration.header.stamp = ros::Time::now();
        marker_imu_acceleration.ns = "my_namespace";
        marker_imu_acceleration.id = 1;
        marker_imu_acceleration.type = visualization_msgs::Marker::ARROW;
        marker_imu_acceleration.action = visualization_msgs::Marker::ADD;
        marker_imu_acceleration.scale.x = 0.4;
        marker_imu_acceleration.scale.y = 0.7;
        marker_imu_acceleration.color.a = 0.75;
        marker_imu_acceleration.color.r = 0;
        marker_imu_acceleration.color.g = 1;
        marker_imu_acceleration.color.b = 0;
        marker_imu_acceleration.lifetime = ros::Duration(0.3);
        marker_imu_acceleration.points.resize(2);
        marker_imu_acceleration.points[0].x = 0;
        marker_imu_acceleration.points[0].y = 0;
        // std::cout << "course_angle_rad: " << course_angle_rad*180/3.14 << std::endl;
        // marker_imu_acceleration.points[1].x = (0 + imu_acceleration_x * cosf(course_angle_rad) + imu_acceleration_y * sinf(course_angle_rad))*10;
        // marker_imu_acceleration.points[1].y = (0 + imu_acceleration_y * cosf(course_angle_rad) - imu_acceleration_x * sinf(course_angle_rad))*10;
        marker_imu_acceleration.points[1].x = (0 + imu_acceleration_x)*10;
        marker_imu_acceleration.points[1].y = imu_acceleration_y*10;
        MarkerArray_imu_acceleration.markers.push_back(marker_imu_acceleration);


        // visualization_msgs::Marker gps_route;
        // gps_route.header.frame_id = "fusionData";
        // gps_route.header.stamp = ros::Time::now();
        // gps_route.ns = "my_namespace";      
        // gps_route.id = 777;
        // gps_route.action= visualization_msgs::Marker::ADD;
        // gps_route.type = visualization_msgs::Marker::LINE_STRIP;
        // gps_route.pose.orientation.x = 0.0;
        // gps_route.pose.orientation.y = 0.0;
        // gps_route.pose.orientation.z = 0.0;
        // gps_route.pose.orientation.w = 1.0;
        // gps_route.scale.x = 0.2;
        // gps_route.scale.y = 0.2;
        // gps_route.scale.z = 0.2;
        // gps_route.color.a = 1.0; // Don't forget to set the alpha!
        // gps_route.color.r = 1.0;
        // gps_route.color.g = 0.5;
        // gps_route.color.b = 0.2;
        // geometry_msgs::Point p_route;       
        // if(flag_gps_frist_frame == true){
        //     p_route.x = 0;
        //     p_route.y = 0;
        //     p_route.z = 0;
        //     gps_route.points.push_back(p_route);                    
        //     p_route.x = route_x;
        //     p_route.y = route_y;
        //     p_route.z = 0;
        //     gps_route.points.push_back(p_route);
        // }else{
        //     p_route.x = glb_rviz_route_x;
        //     p_route.y = glb_rviz_route_y;
        //     p_route.z = 0;
        //     gps_route.points.push_back(p_route);    
        //     p_route.x = route_x;
        //     p_route.y = route_y;
        //     p_route.z = 0;
        //     gps_route.points.push_back(p_route);            
        // }
        // glb_rviz_route_x = route_x;
        // glb_rviz_route_x = route_y;
        visualization_msgs::MarkerArray cam_boxes_array;
        visualization_msgs::Marker cam_bbox;
        for (size_t i = 0; i < _obs_track.size(); i++)
        {
            if(_obs_track[i].get_obs_radar()->_sensor_type !=0 && _obs_track[i].get_obs_radar()->_sensor_type !=7 ){

                if(_obs_track[i].get_obs_radar()->flag_repeat > 1){
                    cam_bbox.header.frame_id = "fusionData";
                    cam_bbox.header.stamp = ros::Time::now();
                    cam_bbox.ns = "my_namespace";
                    cam_bbox.id = i;
                    cam_bbox.type = visualization_msgs::Marker::CUBE;
                    cam_bbox.action = visualization_msgs::Marker::ADD;
                    cam_bbox.color.a = 0.75;
                    if(_obs_track[i].get_obs_radar()->_object_type == 1){
                        cam_bbox.color.r = 0;
                        cam_bbox.color.g = 1;
                        cam_bbox.color.b = 0;
                    }else{
                        cam_bbox.color.r = 1;
                        cam_bbox.color.g = 0;
                        cam_bbox.color.b = 1;
                    }
                    cam_bbox.lifetime = ros::Duration(0.3);
                    geometry_msgs::PoseWithCovariance cam_ssss = getEulerAnglesToQuaternion(_obs_track[i].get_obs_radar()->_object_type,\
                                                                    _obs_track[i].get_obs_radar()->_obs_position,\
                                                                            _obs_track[i].get_obs_radar()->_Box3d,\
                                                                    _obs_track[i].get_obs_radar()->_obs_theta);
                    cam_bbox.pose = cam_ssss.pose;
                    cam_bbox.scale.x = _obs_track[i].get_obs_radar()->_Box3d.x;
                    cam_bbox.scale.y = _obs_track[i].get_obs_radar()->_Box3d.y;
                    cam_bbox.scale.z = _obs_track[i].get_obs_radar()->_Box3d.z;
                    //std::cout << " x " << _obs_track[i].get_obs_radar()->object_box_size.x << " y " << _obs_track[i].get_obs_radar()->object_box_size.y << std::endl;
                    if (cam_bbox.scale.x == 0.0){
                        cam_bbox.scale.x = 0.01;
                    }
                    if (cam_bbox.scale.y == 0.0){
                        cam_bbox.scale.y = 0.01;
                    }
                    cam_bbox.scale.z = 1;
                    cam_boxes_array.markers.push_back(cam_bbox); 
                }
            }
        }
        pub_cam_bbox.publish(cam_boxes_array);


        pub_fusion_markerArray.publish(array);
        pub_fusion_text.publish(array_text);
        pub_fusion_region.publish(line_strip);
        pub_contour.publish(contour_lines);
        pub_box.publish(object_boxes);
        pub_velocity.publish(object_velocity);
        pub_imu_acceleration.publish(MarkerArray_imu_acceleration);
        // pub_gps_route.publish(gps_route);
    }

    static geometry_msgs::Quaternion EulerAnglesToQuaternion(float Roll, float Pitch, float Yaw){
        float cosRoll = cosf(Roll * 0.5f);
        float sinRoll = sinf(Roll * 0.5f);
        float cosPitch = cosf(Pitch * 0.5f); 
        float sinPitch = sinf(Pitch * 0.5f); 
        float cosHeading = cosf(Yaw * 0.5f); 
        float sinHeading = sinf(Yaw * 0.5f); 
        geometry_msgs::Quaternion q;
        q.w = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading; 
        q.x = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading; 
        q.y = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading; 
        q.z = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;
        return q;
    }

    static float QuaternionToEulerAngles(geometry_msgs::Quaternion q){
        float roll = atan2f(2.f * (q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z); 
        float pitch = asinf(2.f * (q.w*q.y - q.x*q.z)); 
        float yaw = atan2f(2.f * (q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
        return roll,pitch,yaw;
    }

    static geometry_msgs::PoseWithCovariance getEulerAnglesToQuaternion(int type,\
        OBFSVector3d _obs_position, geometry_msgs::Point _Box3d, float _obj_ori) {

        geometry_msgs::PoseWithCovariance _Box3d_center;
        /*_Box3d_center.pose.orientation*/
        _Box3d_center.pose.orientation = EulerAnglesToQuaternion( 0, 0, _obj_ori);
        /*_Box3d_center.pose.position*/
        float theta = _obj_ori;
        if(theta>PI/2) {    theta = PI - theta;    }
        if(theta<-PI/2){    theta = -theta- PI;    }
        // _Box3d.x = 4; _Box3d.y = 2; _Box3d.z = 1;
        if(type == 1){
            _Box3d_center.pose.position.x = _obs_position._x + _obs_position._x/fabs(_obs_position._x)*_Box3d.x*cosf(theta)/2;
            _Box3d_center.pose.position.y = _obs_position._y + _Box3d.y*sinf(theta)/2;
            _Box3d_center.pose.position.z = _Box3d.z;
        }else{
            _Box3d_center.pose.position.x = _obs_position._x;
            _Box3d_center.pose.position.y = _obs_position._y;
            _Box3d_center.pose.position.z = _Box3d.z;
        }
        
        return  _Box3d_center;
    }

    static void publish_final_fusion(){
        std::vector<tsinghua::dias::fusion::OBSFUncertaintyTrack> _obs_track;
        _obs_track=_Engine.get_tracks();
        int obs_array_sensor_type = _Engine.get_obs_array_sensor_type();
        // if(obs_array_sensor_type == PUBLISHER_SENSOR_TYPE){ 
        if(true){           
            ros_fusion::TrackArray track_array;         
            track_array.header.stamp = ros::Time::now();
            track_array.header.frame_id ="jxy_ww"; 
            for (size_t i = 0; i < _obs_track.size(); i++)
            {
                ros_fusion::track tracks;
                if(_obs_track[i].get_obs_radar()->flag_repeat < 2){  //flag_repeat 初始化为　0
                    continue;
                }

                tracks.acceleration = _obs_track[i].get_obs_radar()->_acceleration;

                tracks.header.stamp = _obs_track[i].get_obs_radar()->_sensor_header.stamp;
                tracks.id = _obs_track[i].get_obs_radar()->_obs_id;
                tracks.flag_repeat = _obs_track[i].get_obs_radar()->flag_repeat;
                tracks.object_type = _obs_track[i].get_obs_radar()->_object_type;
                tracks.tracks_lane = _obs_track[i].get_obs_radar()->_tracks_lane;
                tracks.obs_position.x = _obs_track[i].get_obs_radar()->_obs_position._x;
                tracks.obs_position.y = _obs_track[i].get_obs_radar()->_obs_position._y;
                tracks.obs_position.z = _obs_track[i].get_obs_radar()->_obs_position._z;
                
                tracks.velocity.x = _obs_track[i].get_obs_radar()->_velocity._x;
                tracks.velocity.y = _obs_track[i].get_obs_radar()->_velocity._y;
                tracks.velocity.z = _obs_track[i].get_obs_radar()->_velocity._z;
                // tracks.object_box_center = _obs_track[i].get_obs_radar()->object_box_center; 
                // tracks.object_box_size.x = _obs_track[i].get_obs_radar()->object_box_size.x;
                // tracks.object_box_size.y = _obs_track[i].get_obs_radar()->object_box_size.y;
                // tracks.object_box_size.z = _obs_track[i].get_obs_radar()->object_box_size.z;
                tracks.object_box_center = getEulerAnglesToQuaternion(_obs_track[i].get_obs_radar()->_object_type,\
                                                                    _obs_track[i].get_obs_radar()->_obs_position,\
                                                                            _obs_track[i].get_obs_radar()->_Box3d,\
                                                                    _obs_track[i].get_obs_radar()->_obs_theta); 
                tracks.object_box_size = _obs_track[i].get_obs_radar()->_Box3d;
                tracks.obj_ori = _obs_track[i].get_obs_radar()->_obs_theta;  // 0:180 ; 0:-180
                tracks.Box3D_size = _obs_track[i].get_obs_radar()->_Box3d; // x=length y=width z=height               
                tracks.Box3D_center =  getEulerAnglesToQuaternion(_obs_track[i].get_obs_radar()->_object_type,\
                                                                    _obs_track[i].get_obs_radar()->_obs_position,\
                                                                            _obs_track[i].get_obs_radar()->_Box3d,\
                                                                    _obs_track[i].get_obs_radar()->_obs_theta);
                tracks.contour_points.clear();
                for(size_t j=0; j<_obs_track[i].get_obs_radar()->contour_points.size(); j++){
                    geometry_msgs::Point contour_point;
                    contour_point = _obs_track[i].get_obs_radar()->contour_points[j];
                    tracks.contour_points.push_back(contour_point);
                }


                int l_hist=_obs_track[i].get_obs_radar()->_hist_states.size();
                if (_obs_track[i].get_obs_radar()->_hist_states.size()==0){
                    tracks.acceleration.x=0;
                }else{
                    std::vector<float> acc_valid ;
                    if (l_hist>3)
                    {
                    
                        for (int ii = 0 ; ii< l_hist-2 ; ii++)
                        {
                            float acc = (_obs_track[i].get_obs_radar()->_hist_states[ii]._self_velocity._x + _obs_track[i].get_obs_radar()->_hist_states[ii]._velocity._x \
                            - _obs_track[i].get_obs_radar()->_hist_states[ii+2]._velocity._x - _obs_track[i].get_obs_radar()->_hist_states[ii+2]._self_velocity._x) \
                            /(_obs_track[i].get_obs_radar()->_hist_states[ii]._header - _obs_track[i].get_obs_radar()->_hist_states[ii+2]._header) ;

                            if (fabs(acc)<5)
                            {
                                acc_valid.push_back(acc);
                                // std::cout<< "valid acc: "<< acc << std::endl ;
                            }
                        }                        
                    }
                    else
                    {
                        for (int ii=0 ; ii<l_hist -1 ; ii++)
                        {
                            float acc = (_obs_track[i].get_obs_radar()->_hist_states[ii]._self_velocity._x + _obs_track[i].get_obs_radar()->_hist_states[ii]._velocity._x \
                            - _obs_track[i].get_obs_radar()->_hist_states[ii+1]._velocity._x - _obs_track[i].get_obs_radar()->_hist_states[ii+1]._self_velocity._x) \
                            /(_obs_track[i].get_obs_radar()->_hist_states[ii]._header - _obs_track[i].get_obs_radar()->_hist_states[ii+1]._header) ;

                            if (fabs(acc)<5)
                            {
                                acc_valid.push_back(acc);
                                // std::cout<< "valid acc: "<< acc << std::endl ;
                            }
                        }
                    }

                    
                    float final_acc = 0 ;

                    if (acc_valid.size()>0)
                    {
                        float max = acc_valid[0] ;
                        float min = max ;
                        if (acc_valid.size()>1)
                        {
                            for (int i=1 ; i<acc_valid.size(); i++)
                            {
                                if (acc_valid[i]>max)
                                {
                                    max = acc_valid[i];
                                }

                                if (acc_valid[i]<min)
                                {
                                    min = acc_valid[i];
                                }
                            }
                            final_acc = (float)(std::accumulate(std::begin(acc_valid), std::end(acc_valid),0.0)-max-min)/(acc_valid.size()-2) ;
                        }
                        else if (acc_valid.size()==1)
                        {
                            final_acc = acc_valid[0];
                        }
                       
                    }

                    acc_valid.clear();

                    tracks.acceleration.x = final_acc - imu_acceleration_x;
    
                }

                track_array.tracks.push_back(tracks);
                

            } 

            track_array.self_velocity.x = glb_twist_linear_x*cosf(course_angle_rad)+glb_twist_linear_y*sinf(course_angle_rad);
            track_array.self_velocity.y = glb_twist_linear_y*cosf(course_angle_rad)-glb_twist_linear_x*sinf(course_angle_rad);
            track_array.self_velocity.z = 0;  
            track_array.gps_route.x = route_x; 
            track_array.gps_route.y = route_y;
            track_array.gps_route.z = 0;
            track_array.imu_pose.x = imu_roll; 
            track_array.imu_pose.y = imu_pitch;
            track_array.imu_pose.z = imu_yaw;
            track_array.self_acceleration.x = imu_acceleration_x;
            track_array.self_acceleration.y = imu_acceleration_y;
            track_array.self_acceleration.z = 0.0;
            track_array.heading_angle = course_angle_rad;                   
            pub_final_fusion.publish(track_array);
            // std::cout <<track_array.header.stamp<<" "<<track_array.self_velocity.x << " "<< track_array.self_acceleration.x << " "<<track_array.self_acceleration.y << std::endl ;
            // std::cout <<track_array.self_velocity.x << " "<< track_array.self_acceleration.x  << std::endl ;
        }
    }


    static void run(int argc, char **argv){
        ros::init(argc, argv, "fusion");

        ros::NodeHandle nh;
        std::string obstacle_cloud_msg_name = "/cloud1";
        std::string obstacle_velodyne_msg_name = "/velodyne_points";
        std::string obstacle_front_radar_msg_name = "/lrr_topic1";
        std::string obstacle_right_front_radar_msg_name = "/srr_topic1";
        std::string obstacle_left_front_radar_msg_name = "/srr_topic3";
        std::string obstacle_msg_name = "/DriveObsArray";
        std::string obstacle_camera_bbox_msg_name = "/darknet_ros/bounding_boxes";
        

        ros::NodeHandle launch_nh("~");
        launch_nh.getParam("lidar_right_slope", lidar_right_slope);
        launch_nh.getParam("lidar_left_slope", lidar_left_slope);
        launch_nh.getParam("lidar_x_max_dist", lidar_x_max_dist);
        launch_nh.getParam("B_lidar_right_slope", B_lidar_right_slope);
        launch_nh.getParam("B_lidar_left_slope", B_lidar_left_slope);
        launch_nh.getParam("B_lidar_x_max_dist", B_lidar_x_max_dist);
        launch_nh.getParam("SRR_FR_ANGLE", SRR_FR_ANGLE);
        launch_nh.getParam("SRR_FL_ANGLE", SRR_FL_ANGLE);
        launch_nh.getParam("SRR_BR_ANGLE", SRR_BR_ANGLE);
        launch_nh.getParam("SRR_BL_ANGLE", SRR_BL_ANGLE);
        launch_nh.getParam("LRR_F_ANGLE", LRR_F_ANGLE);
        launch_nh.getParam("LRR_B_ANGLE", LRR_B_ANGLE);
        launch_nh.getParam("LRR_F_pos_x", LRR_F_pos_x);
        launch_nh.getParam("LRR_F_pos_y", LRR_F_pos_y);
        launch_nh.getParam("LRR_B_pos_x", LRR_B_pos_x);
        launch_nh.getParam("LRR_B_pos_y", LRR_B_pos_y);
        launch_nh.getParam("SRR_FR_pos_x", SRR_FR_pos_x);
        launch_nh.getParam("SRR_FR_pos_y", SRR_FR_pos_y);
        launch_nh.getParam("SRR_FL_pos_x", SRR_FL_pos_x);
        launch_nh.getParam("SRR_FL_pos_y", SRR_FL_pos_y);
        launch_nh.getParam("SRR_BL_pos_x", SRR_BL_pos_x);
        launch_nh.getParam("SRR_BL_pos_y", SRR_BL_pos_y);
        launch_nh.getParam("SRR_BR_pos_x", SRR_BR_pos_x);
        launch_nh.getParam("SRR_BR_pos_y", SRR_BR_pos_y);       
        launch_nh.getParam("PUBLISHER_SENSOR_TYPE", PUBLISHER_SENSOR_TYPE);
        launch_nh.getParam("LIDAR_B_ANGLE", LIDAR_B_ANGLE);
        _Engine.init_param(lidar_right_slope, lidar_left_slope, lidar_x_max_dist,\
             B_lidar_right_slope, B_lidar_left_slope, B_lidar_x_max_dist,\
             SRR_FR_ANGLE, SRR_FL_ANGLE , SRR_BR_ANGLE, SRR_BL_ANGLE,\
             LRR_F_ANGLE, LRR_B_ANGLE, LRR_F_pos_x, LRR_F_pos_y,\
             LRR_B_pos_x, LRR_B_pos_y, SRR_FR_pos_x, SRR_FR_pos_y,\
             SRR_FL_pos_x, SRR_FL_pos_y, SRR_BL_pos_x, SRR_BL_pos_y,\
             SRR_BR_pos_x, SRR_BR_pos_y, PUBLISHER_SENSOR_TYPE,LIDAR_B_ANGLE);

        std::cout << "begin the fusion" << std::endl;
       

        Eigen::Matrix3d m;
        m = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(M_PI,  Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(0.38, Eigen::Vector3d::UnitZ());

        Eigen::Vector3d temp(0.3,0,0);    
        Eigen::Affine3d transform_temp;
        transform_temp.matrix() = Eigen::Matrix4d::Identity();
        // transform_temp.matrix().block<3,3>(0,0) = m;
        // transform_temp.matrix().block<3,1>(0,3) = temp;

        // transform_temp.matrix() <<    -0.932096,     0.362211, -2.55193e-08,      -2.9746,
        //                         -0.362211,    -0.932096,  5.52892e-08,    -0.178792,
        //                         -3.76009e-09,  6.07782e-08,            1,        -1.52,
        //                                 0,            0,            0,            1;
        transform.matrix() = transform_temp.matrix().inverse();
        // std::cout << "get the inverse" << std::endl;

        _Engine.set_control_mode(f_control_mode, b_control_mode);
/*
        ros::NodeHandle private_node_handle_("~");
        private_node_handle_.getParam("obstacle_3d_msg_name", obstacle_3d_msg_name);
        private_node_handle_.getParam("obstacle_radar_msg_name", obstacle_radar_msg_name);
        private_node_handle_.getParam("obstacle_msg_name", obstacle_msg_name);
*/

        // g_obs_publisher = nh.advertise<ros_fusion::DriveObsArray>(
        //     obstacle_msg_name,
        //     1000);

        // ros::Subscriber lidar_obs3d_sub = nh.subscribe(
        //     obstacle_cloud_msg_name,
        //     1000,
        //     callback_cloud_visualization);

        // ros::Subscriber velodyne_obs3d_sub = nh.subscribe(
        //     obstacle_velodyne_msg_name,
        //     1000,
        //     callback_velodyne_visualization);            
        // ros::Rate loop_rate(50);
        // while (ros::ok())  
        // {
        //     ros::spinOnce();
        /*front_lidar*/
        ros::Subscriber lidar_front_obs3d_sub = nh.subscribe("objects1", 1, obs_callback_local_front_lidar_online);
        /*rear_lidar*/
        ros::Subscriber lidar_rear_obs3d_sub = nh.subscribe("objects2", 1, obs_callback_local_rear_lidar_online);
        /*front_radar*/
        ros::Subscriber radar_front_obs3d_sub = nh.subscribe("lrr_topic2", 1, obs_callback_local_front_radar_online);
        /*rear_radar*/
        
        
        ros::Subscriber radar_rear_obs3d_sub = nh.subscribe("lrr_topic1", 1, obs_callback_local_rear_radar_online);
        /*right_front_radar*/
        ros::Subscriber radar_right_front_obs3d_sub = nh.subscribe("srr_topic3", 1, obs_callback_local_right_front_radar_online);
        /*right_rear_radar*/
        ros::Subscriber radar_right_rear_obs3d_sub = nh.subscribe("srr_topic1", 1, obs_callback_local_right_rear_radar_online);
        /*left_front_radar*/
        ros::Subscriber radar_left_front_obs3d_sub = nh.subscribe("srr_topic4", 1, obs_callback_local_left_front_radar_online);
        /*left_rear_radar*/
        ros::Subscriber radar_left_rear_obs3d_sub = nh.subscribe("srr_topic2", 1, obs_callback_local_left_rear_radar_online);
        /*gps_vel*/
        ros::Subscriber gps_vel_sub = nh.subscribe("gps/vel", 1, obs_callback_gps_vel_online);
        /*imu_data*/
        ros::Subscriber imu_data_sub = nh.subscribe("imu/data", 1, obs_callback_imu_data_online);
        /*gps_fix*/
        // ros::Subscriber gps_fix_sub = nh.subscribe("navsat/fix", 1, obs_callback_gps_fix_online); //navsat
        /*camera_data*/
        ros::Subscriber bbox_data_sub = nh.subscribe(obstacle_camera_bbox_msg_name, 1, obs_callback_camera_bbox_online);
        /*lane_data*/
        ros::Subscriber lane_data_sub =  nh.subscribe("lane_data", 1, obs_callback_lane_data_online);
        // _visualizer.initialize_pcl_visualizer();
        //     loop_rate.sleep();
        // }
        pub_fusion_markerArray = nh.advertise<visualization_msgs::MarkerArray>("fusion_markerArray", 1);
        pub_fusion_text = nh.advertise<visualization_msgs::MarkerArray>("fusion_label", 1);

        radar_raw_pub=nh.advertise<visualization_msgs::MarkerArray>("/radar_before_fusion", 1);

        // pub_lidar_box = nh.advertise<visualization_msgs::MarkerArray>("lidar_box", 1);
        // pub_lidar_velocity = nh.advertise<visualization_msgs::MarkerArray>("lidar_velocity", 1);
        // pub_lidar_contour = nh.advertise<visualization_msgs::MarkerArray>("lidar_contour", 1);
        // pub_lidar_markerArray = nh.advertise<visualization_msgs::MarkerArray>("lidar_markerArray", 1);
        // pub_lidar_text = nh.advertise<visualization_msgs::MarkerArray>("lidar_label", 1);
        // pub_camera_text = nh.advertise<visualization_msgs::MarkerArray>("camera_marker", 1);
        // pub_camera_markerArray = nh.advertise<visualization_msgs::MarkerArray>("camera_markerArray", 1);
        
        pub_fusion_region = nh.advertise<visualization_msgs::Marker>("fusion_region", 1);
        pub_box = nh.advertise<visualization_msgs::MarkerArray>("fusion_box", 1);
        pub_contour = nh.advertise<visualization_msgs::MarkerArray>("fusion_contour", 1);
        pub_velocity = nh.advertise<visualization_msgs::MarkerArray>("fusion_velocity", 1);
        pub_imu_acceleration = nh.advertise<visualization_msgs::MarkerArray>("fusion_imu_acceleration", 1);
        // pub_gps_route = nh.advertise<visualization_msgs::Marker>("fusion_gps_route", 1);
        lane_pub = nh.advertise<visualization_msgs::Marker>("fusion_lane", 1);

        pub_cam_bbox = nh.advertise<visualization_msgs::MarkerArray>("fusion_cam_bbox", 1);

        pub_final_fusion = nh.advertise<ros_fusion::TrackArray>("final_fusion", 1000);

        ros::spin();
    }
};

}//fusion
}//car
}//tsinghua

#endif
