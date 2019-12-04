// @brief: Enter fusion
// objective: Collect all sensor data, enter the fusion module
//Author: Chunlei YU. Contact me at: yuchunlei@mail.tsinghua.edu.cn

#ifndef CAR_OBSF_H
#define CAR_OBSF_H



#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
#include "OpenICV/Core/icvTime.h"
#include "OpenICV/Basis/icvStructureData.hxx"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Basis/icvSemaphore.h"
#include "OpenICV/structure/icvCvMatData.hxx"

#include "OpenICV/structure/structureCanTuan.h"
#include "OpenICV/structure/structureVehinfo.h"
#include "OpenICV/structure/structureIMU.h"
#include "OpenICV/structure/structureFusionData.h"
#include "OpenICV/structure/structureLaneData.h"
#include "OpenICV/Net/icvUdpReceiverSource.h"
#include "OpenICV/structure/RadarRecordReplay.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <opencv/highgui.h>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>   
#include <opencv2/highgui/highgui.hpp>  


#include "filters/obsf_uncertainty_fusion_engine.h"
#include "objects/obsf_header.h"

#define SRR_FR_ANGLE -30.0*tsinghua::dias::fusion::PI/180 
#define SRR_FL_ANGLE 28.0*tsinghua::dias::fusion::PI/180

#define SRR_BR_ANGLE 35.0*tsinghua::dias::fusion::PI/180
#define SRR_BL_ANGLE 30.0*tsinghua::dias::fusion::PI/180

#define LRR_F_ANGLE  2.0*tsinghua::dias::fusion::PI/180
#define LRR_B_ANGLE  180*tsinghua::dias::fusion::PI/180

#define LRR_F_pos_x 3.7
#define LRR_F_pos_y 0.0

#define LRR_B_pos_x -1.03
#define LRR_B_pos_y 2.0

#define SRR_FR_pos_x 3.68
#define SRR_FR_pos_y -0.68

#define SRR_FL_pos_x 3.65
#define SRR_FL_pos_y 0.7

#define SRR_BL_pos_x -0.94
#define SRR_BL_pos_y -0.68

#define SRR_BR_pos_x -0.91
#define SRR_BR_pos_y 0.72
using namespace icv;
using namespace core;
using namespace std;

typedef data::icvStructureData<Imu> icvImu;
typedef data::icvStructureData<RadarLongOut>    icvlrr ;
typedef data::icvStructureData<RadarSideOut>    icvsrr ;
typedef data::icvStructureData<NavSatFix>    icvNavSatFix;
typedef data::icvStructureData<TwistWithCovarianceStamped>    icvTwistWithCovarianceStamped;
typedef data::icvStructureData<Odometry>    icvOdometry;
typedef data::icvStructureData<TrackArray> icvfusion ; 
typedef data::icvStructureData<BoundingBoxes2d> icvbbox2d ;
typedef data::icvStructureData<ld_Frame> icvlane;


class SensorFusion: public icvFunction
{
public:
    SensorFusion(icv_shared_ptr<const icvMetaData> info) : icvFunction(12, 0, info) 
    {
        Eigen::Matrix3d m;
        m = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(M_PI,  Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(0.38, Eigen::Vector3d::UnitZ());

        Eigen::Vector3d temp(0.3,0,0);    
        Eigen::Affine3d transform_temp;
        transform_temp.matrix() = Eigen::Matrix4d::Identity();
        transform.matrix() = transform_temp.matrix().inverse();
        timesync_=new icvSemaphore(1);
        fusion_output=new icvfusion();
        fusion_visual=new icv::data::icvCvMatData();
        icv_thread loop(bind(&SensorFusion::InnerLoop, this));
    };


    //gps time
    static uint64_t unix2gps(uint64_t unix_time_usec) 
    {
        // const uint64_t leapSeconds = 16;
        const uint64_t leapSeconds =
        unix_time_usec < uint64_t(1435708799) * 1e6 ? 16 : 17;
        int64_t gps = unix_time_usec - (uint64_t(315964800) - leapSeconds) * 1e6;
        return gps > 0 ? gps : 0;
    }

    
    // coordinnate change
    static void Data_Projection(tsinghua::dias::fusion::OBSFObs3d &obs_tmp)
    {   
        
        double c= 0.66;
        double omega =(obs_tmp._obs_position._x-LRR_F_pos_x) *-0.00059703 +obs_tmp._obs_position._y *-0.000050721 + c*0.00010254-0.0016;
        obs_tmp._v_center= ((obs_tmp._obs_position._x-LRR_F_pos_x)*-0.22384 + obs_tmp._obs_position._y* 0.24574 +c*0.045906 -0.5648) / omega ;
        obs_tmp._u_center= ((obs_tmp._obs_position._x-LRR_F_pos_x) * -0.11899 +obs_tmp._obs_position._y * -0.026076 +c*0.28690-0.8252) / omega ;       
    }
    static bool LessSort(ld_Coeff a,ld_Coeff b){
        return a.d > b.d;
    }   

    void obs_callback_lane_data_online(const ld_Frame& msg){
        std::vector<ld_Coeff> lane_sequence = msg.lane_Coeff;
        sort(lane_sequence.begin(),lane_sequence.end(),LessSort);
        std::vector<int> AutoDrive_lane;
        int lane_sequence_number = 0;
    
        for(int i=0; i<lane_sequence.size(); i++){
            if(lane_sequence[i].d<=0){
                lane_sequence_number = i;
                break;
            }   
        }

        //get the current lane which the driving vehicle is in.
        for(int i=1; i<lane_sequence.size(); i++){
            AutoDrive_lane.push_back(lane_sequence_number-i); 
        }
        double lane_time_stamp = SyncClock::time_s();
        _Engine.set_lane_detection_data(lane_sequence, AutoDrive_lane, lane_time_stamp);
      
    } 
        
    //front radar object info

     void obs_callback_local_front_radar_online(const RadarLongOut &msg) 
     {
        // std::cout << "Now we enter the front radar callback" << std::endl;

        now_time = msg.header_.stamp;;

        uint64_t unix_time_usec = static_cast<uint64_t>(now_time * 1e6);
        uint64_t gps_time_usec = unix2gps(unix_time_usec);
        uint64_t sec =  gps_time_usec / 1000000;
        uint64_t nsec =  (gps_time_usec % 1000000) * 1000;

        //Eigen::Matrix4d radar_pose = Eigen::Matrix4d::Identity();
        // now, the transform is from the radar point to cloud coordinate
        Eigen::Matrix4d radar_pose = transform.matrix();            

        tsinghua::dias::fusion::OBSFHeader header;
        header._seq = msg.header_.seq;
        header._stamp._sec = sec;
        header._stamp._nsec = nsec;

        tsinghua::dias::fusion::OBSFObs3dFrame obs_3d_frame;
        obs_3d_frame.set_header(header);        

        std::vector<tsinghua::dias::fusion::OBSFObs3d> obs_3d_obstacles;
        std::set<int> current_ids;

        for (int i = 0; i < msg.targetnum_; i++)
        {
            if(msg.alldata_[i].x>0){
                tsinghua::dias::fusion::OBSFObs3d obs_3d;
                obs_3d._header._seq = msg.header_.seq;
                obs_3d._header._stamp._sec = sec;
                obs_3d._header._stamp._nsec = nsec;
    //            obs_3d._header._frame_id = msg.header_.frame_id;
                obs_3d._sensor_header =  msg.header_;
                obs_3d._obs_id = msg.alldata_[i].id;
                obs_3d.flag_repeat = 0;
                obs_3d.rangex_rms = msg.alldata_[i].rangex_rms;                
                obs_3d.speedx_rms = msg.alldata_[i].speedx_rms;                
                obs_3d.accx_rms = msg.alldata_[i].accx_rms;
                obs_3d.rangey_rms = msg.alldata_[i].rangey_rms;
                obs_3d.speedy_rms = msg.alldata_[i].speedy_rms;
                obs_3d.accy_rms = msg.alldata_[i].accy_rms;
                obs_3d.rangez_rms = 7.00f; 
                obs_3d.speedz_rms = 100.00f;
                obs_3d._sensor_type = 0;
                obs_3d._object_type = 0;
                
                float _obs_x=msg.alldata_[i].x+LRR_F_pos_x;
                float _obs_y=msg.alldata_[i].y+LRR_F_pos_y;
                Eigen::Matrix<double, 4, 1> location_r;
                Eigen::Matrix<double, 4, 1> location_w;
                double local_pi = tsinghua::dias::fusion::PI;

                location_r << _obs_x,
                            _obs_y,
                            0.0,
                            1.0;
            
                location_w = radar_pose * location_r;
                tsinghua::dias::fusion::OBFSVector3d point;
                point._x = location_w[0];
                point._y = location_w[1];
                point._z = 0.66;
            
                obs_3d._obs_position = point;
                obs_3d._obs_theta = 0.0;
                Eigen::Matrix<double, 4, 1> velocity_r;

                velocity_r << msg.alldata_[i].relspeedx,
                            msg.alldata_[i].relspeedy,
                            0.0,
                            1.0;

                obs_3d._velocity._x = msg.alldata_[i].relspeedx;
                obs_3d._velocity._y = msg.alldata_[i].relspeedy;
                obs_3d._velocity._z = 0.0;
                obs_3d._length = 1.0;
                obs_3d._width = 1.0;
                obs_3d._height = 1.0;
                obs_3d._polygon_points.push_back(point);
                obs_3d._life = 0;
                obs_3d._classification = 0;
                obs_3d._self_velocity._x = glb_twist_linear_x*cosf(course_angle_rad)+glb_twist_linear_y*sinf(course_angle_rad);
                obs_3d._self_velocity._y = glb_twist_linear_y*cosf(course_angle_rad)-glb_twist_linear_x*sinf(course_angle_rad);
                obs_3d._self_velocity._z = glb_twist_linear_z;
                obs_3d._course_angle_rad = course_angle_rad;

                obs_3d.objProbExist = msg.alldata_[i].objProbExist;
                obs_3d.objDynProp = msg.alldata_[i].objDynProp;
                obs_3d.objMeasState = msg.alldata_[i].objMeasState;
                obs_3d.obj_amp = msg.alldata_[i].obj_amp;

                Data_Projection(obs_3d);

                if((obs_3d.objMeasState != 2) || (obs_3d.objProbExist<5) || (obs_3d.obj_amp < 0)){
                    continue;
                }
                obs_3d.objClass = msg.alldata_[i].objClass;
                obs_3d_obstacles.push_back(obs_3d);
            }
        }

        if (obs_3d_obstacles.size() != 0){
            obs_3d_frame.set_obstacles(obs_3d_obstacles);
            _Engine.process_radar_frame(obs_3d_frame);
            publish_final_fusion();
        }
    }

     void obs_callback_local_right_front_radar_online(const RadarSideOut &msg){

        // std::cout << "Now we enter the right front radar callback" << std::endl;

        now_time = msg.header_.stamp;
        

        uint64_t unix_time_usec = static_cast<uint64_t>(now_time * 1e6);
        uint64_t gps_time_usec = unix2gps(unix_time_usec);
        uint64_t sec =  gps_time_usec / 1000000;
        uint64_t nsec =  (gps_time_usec % 1000000) * 1000;

        //Eigen::Matrix4d radar_pose = Eigen::Matrix4d::Identity();
        // now, the transform is from the radar point to cloud coordinate
        Eigen::Matrix4d radar_pose = transform.matrix();    
        

        tsinghua::dias::fusion::OBSFHeader header;
        header._seq = msg.header_.seq;
        header._stamp._sec = sec;
        header._stamp._nsec = nsec;

        tsinghua::dias::fusion::OBSFObs3dFrame obs_3d_frame;
        obs_3d_frame.set_header(header);        

        std::vector<tsinghua::dias::fusion::OBSFObs3d> obs_3d_obstacles;
        std::set<int> current_ids;

        for (size_t i = 0; i < msg.targetnum_; i++)
        {
            if(!(msg.alldata_[i].x==0 && msg.alldata_[i].y==0)){

                tsinghua::dias::fusion::OBSFObs3d obs_3d;
                obs_3d._header._seq = msg.header_.seq;
                obs_3d._header._stamp._sec = sec;
                obs_3d._header._stamp._nsec = nsec;
               // obs_3d._header._frame_id = msg.header_.frame_id;
                obs_3d._sensor_header =  msg.header_;
                obs_3d._obs_id = msg.alldata_[i].id;
                obs_3d.flag_repeat = 0;
                obs_3d._sensor_type = 1;
                obs_3d._object_type = 0;
                Eigen::Matrix<double, 4, 1> location_r;
                Eigen::Matrix<double, 4, 1> location_w;
                double local_pi = tsinghua::dias::fusion::PI;
                float _obs_x=msg.alldata_[i].x*cosf(SRR_FR_ANGLE)-msg.alldata_[i].y*sinf(SRR_FR_ANGLE);
                float _obs_y=msg.alldata_[i].y*cosf(SRR_FR_ANGLE)+msg.alldata_[i].x*sinf(SRR_FR_ANGLE);
                _obs_x=_obs_x+SRR_FR_pos_x;
                _obs_y=_obs_y+SRR_FR_pos_y;
                float _obs_vx=msg.alldata_[i].relspeedx*cosf(SRR_FR_ANGLE)-msg.alldata_[i].relspeedy*sinf(SRR_FR_ANGLE);
                float _obs_vy=msg.alldata_[i].relspeedy*cosf(SRR_FR_ANGLE)+msg.alldata_[i].relspeedx*sinf(SRR_FR_ANGLE);

                location_r << _obs_x,
                            _obs_y,
                            0.0,
                            1.0;
            
                location_w = radar_pose * location_r;
                tsinghua::dias::fusion::OBFSVector3d point;
                point._x = location_w[0];
                point._y = location_w[1];
                point._z = 0.0;            
                obs_3d._obs_position = point;

                float track_slope = (point._x - LRR_F_pos_x) / point._y;
                if (point._y < 0){
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
                obs_3d.accx_rms = 14.00f;
                obs_3d.accy_rms = 14.00f;

                obs_3d._obs_theta = 0.0;
                Eigen::Matrix<double, 4, 1> velocity_r;
                velocity_r << _obs_vx,
                            _obs_vy,
                            0.0,
                            1.0;

                obs_3d._velocity._x = 0.0;
                obs_3d._velocity._y = 0.0;
                obs_3d._velocity._z = 0.0;

                obs_3d._length = 1.0;
                obs_3d._width = 1.0;
                obs_3d._height = 1.0;
                obs_3d._polygon_points.push_back(point);
                obs_3d._life = 0;
                obs_3d._classification = 0;
                obs_3d._self_velocity._x = glb_twist_linear_x*cosf(course_angle_rad)+glb_twist_linear_y*sinf(course_angle_rad);
                obs_3d._self_velocity._y = glb_twist_linear_y*cosf(course_angle_rad)-glb_twist_linear_x*sinf(course_angle_rad);
                obs_3d._self_velocity._z = glb_twist_linear_z;
                obs_3d._course_angle_rad = course_angle_rad;

                Data_Projection(obs_3d);
                obs_3d_obstacles.push_back(obs_3d);


            }
        }

        if(obs_3d_obstacles.size()!=0){
            obs_3d_frame.set_obstacles(obs_3d_obstacles);
            _Engine.process_radar_frame(obs_3d_frame);
            publish_final_fusion();
        }

    }

     void obs_callback_local_left_front_radar_online(const RadarSideOut &msg){
        
        // std::cout << "Now we enter the left front radar callback" << std::endl;

        now_time = msg.header_.stamp;;
        
        uint64_t unix_time_usec = static_cast<uint64_t>(now_time * 1e6);
        uint64_t gps_time_usec = unix2gps(unix_time_usec);
        uint64_t sec =  gps_time_usec / 1000000;
        uint64_t nsec =  (gps_time_usec % 1000000) * 1000;

        //Eigen::Matrix4d radar_pose = Eigen::Matrix4d::Identity();
        // now, the transform is from the radar point to cloud coordinate
        Eigen::Matrix4d radar_pose = transform.matrix();    
        

        tsinghua::dias::fusion::OBSFHeader header;
        header._seq = msg.header_.seq;
        header._stamp._sec = sec;
        header._stamp._nsec = nsec;

        tsinghua::dias::fusion::OBSFObs3dFrame obs_3d_frame;
        obs_3d_frame.set_header(header);        

        std::vector<tsinghua::dias::fusion::OBSFObs3d> obs_3d_obstacles;
        std::set<int> current_ids;

        for (size_t i = 0; i < msg.targetnum_; i++)
        {
            if(!(msg.alldata_[i].x==0 && msg.alldata_[i].y==0)){

                tsinghua::dias::fusion::OBSFObs3d obs_3d;
               obs_3d._header._seq = msg.header_.seq;
                obs_3d._header._stamp._nsec = nsec;
            //    obs_3d._header._frame_id = msg.header_.frame_id;
                obs_3d._sensor_header =  msg.header_;
                obs_3d._obs_id = msg.alldata_[i].id;
                obs_3d.flag_repeat = 0;                
                obs_3d._sensor_type = 2;
                obs_3d._object_type = 0;
                Eigen::Matrix<double, 4, 1> location_r;
                Eigen::Matrix<double, 4, 1> location_w;
                double local_pi = tsinghua::dias::fusion::PI;
                float _obs_x=msg.alldata_[i].x*cosf(SRR_FL_ANGLE)-msg.alldata_[i].y*sinf(SRR_FL_ANGLE);
                float _obs_y=msg.alldata_[i].y*cosf(SRR_FL_ANGLE)+msg.alldata_[i].x*sinf(SRR_FL_ANGLE);
                _obs_x=_obs_x+SRR_FL_pos_x;
                _obs_y=_obs_y+SRR_FL_pos_y;
                float _obs_vx=msg.alldata_[i].relspeedx*cosf(SRR_FL_ANGLE)-msg.alldata_[i].relspeedy*sinf(SRR_FL_ANGLE);
                float _obs_vy=msg.alldata_[i].relspeedy*cosf(SRR_FL_ANGLE)+msg.alldata_[i].relspeedx*sinf(SRR_FL_ANGLE);
                location_r << _obs_x,
                            _obs_y,
                            0.0,
                            1.0;
            
                location_w = radar_pose * location_r;
                tsinghua::dias::fusion::OBFSVector3d point;
                point._x = location_w[0];
                point._y = location_w[1];
                point._z = 0.0;            
                obs_3d._obs_position = point;
                float track_slope = (point._x - LRR_F_pos_x) / point._y;
                if (point._y > 0){
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
                obs_3d.accx_rms = 14.00f;
                obs_3d.accy_rms = 14.00f;

                obs_3d._obs_theta = 0.0;
                Eigen::Matrix<double, 4, 1> velocity_r;

                velocity_r << _obs_vx,
                            _obs_vy,
                            0.0,
                            1.0;

                obs_3d._velocity._x = 0.0;
                obs_3d._velocity._y = 0.0;
                obs_3d._velocity._z = 0.0;

                obs_3d._length = 1.0;
                obs_3d._width = 1.0;
                obs_3d._height = 1.0;
                obs_3d._polygon_points.push_back(point);
                obs_3d._life = 0;
                obs_3d._classification = 0;
                obs_3d._self_velocity._x = glb_twist_linear_x*cosf(course_angle_rad)+glb_twist_linear_y*sinf(course_angle_rad);
                obs_3d._self_velocity._y = glb_twist_linear_y*cosf(course_angle_rad)-glb_twist_linear_x*sinf(course_angle_rad);
                obs_3d._self_velocity._z = glb_twist_linear_z;
                obs_3d._course_angle_rad = course_angle_rad;

                Data_Projection(obs_3d);
                
                obs_3d_obstacles.push_back(obs_3d);
            }
        }
        
        if(obs_3d_obstacles.size()!=0){
        obs_3d_frame.set_obstacles(obs_3d_obstacles);
        _Engine.process_radar_frame(obs_3d_frame);
        publish_final_fusion();
        }
    }

     void obs_callback_gps_vel_online(const TwistWithCovarianceStamped& msg) 
    {
        // std::cout << "Now we enter the gps vel callback" << std::endl;
        glb_twist_linear_x = msg.twist.twist.linear.x;
        glb_twist_linear_y = msg.twist.twist.linear.y;
        glb_twist_linear_z = msg.twist.twist.linear.z;
    }

     void obs_callback_imu_data_online(const Imu& msg) 
    {
        // std::cout << "Now we enter the imu data callback" << std::endl;
        float x = msg.orientation.x;
        float y = msg.orientation.y;
        float z = msg.orientation.z;
        float w = msg.orientation.w;
        course_angle_rad = tsinghua::dias::fusion::PI/2 + atan2f(2*(w*z+x*y), w*w+x*x-y*y-z*z);
    }

     void obs_callback_gps_fix_online(const NavSatFix& msg) 
    {
        // std::cout << "Now we enter the gps fix callback" << std::endl;
        double lon, lat, lat_b, lon_b;
        float scale[2], a, f, b, e2, A, B;
        lat=msg.latitude;
        lon=msg.longitude; 
        if(flag_gps_frist_frame == true){
            lat_b = lat;
            lon_b = lon;
            temp_lat = lat;
            temp_lon = lon;
            flag_gps_frist_frame = false;
        }else{
            lat_b=temp_lat;                               
            lon_b=temp_lon;
        }
        a=6378137.0;
        f=298.2572; 
        b=(f - 1) / f * a;
        e2=(a*a - b*b) / (a*a);
        A = a * (1 - e2) / pow((1-e2*pow(sin(lat_b/180.0*tsinghua::dias::fusion::PI),2)),1.5);
        B = a * cos(lat_b/180.0*tsinghua::dias::fusion::PI)/sqrt(1-e2*pow(sin(lat_b/180.0*tsinghua::dias::fusion::PI),2));
        scale[0]=B*1.0/180.0*tsinghua::dias::fusion::PI;
        scale[1]=A*1.0/180.0*tsinghua::dias::fusion::PI;
        route_x = (lon - lon_b) * scale[0];   //转换为坐标
        route_y = (lat - lat_b) * scale[1];
    }

    void obs_callback_camera_bbox2d_online(const BoundingBoxes2d &_objects){
        // std::cout << "Now we enter the bbox callback" << std::endl;
        now_time = _objects.header.stamp;

        uint64_t unix_time_usec = static_cast<uint64_t>(now_time * 1e6);
        uint64_t gps_time_usec = unix2gps(unix_time_usec);
        uint64_t sec = gps_time_usec / 1000000;
        uint64_t nsec = (gps_time_usec % 1000000) * 1000;


        Eigen::Matrix4d radar_pose = Eigen::Matrix4d::Identity();

        tsinghua::dias::fusion::OBSFHeader header;
        header._seq = _objects.header.seq;
        header._stamp._sec = sec;
        header._stamp._nsec = nsec;
        // header._frame_id = _objects.header.frame_id;

        tsinghua::dias::fusion::OBSFObs3dFrame obs_3d_frame;
        obs_3d_frame.set_header(header);

        std::vector<tsinghua::dias::fusion::OBSFObs3d> obs_3d_obstacles;
        std::set<int> current_ids;
        tsinghua::dias::fusion::OBFSVector3d point;

        for (size_t i = 0; i < _objects.bounding_boxes.size(); i++)
        {
            int resize_xmax = (int)(_objects.bounding_boxes[i].xmax+128);
            int resize_ymax = (int)(_objects.bounding_boxes[i].ymax+150);
            int resize_xmin = (int)(_objects.bounding_boxes[i].xmin+128);
            int resize_ymin = (int)(_objects.bounding_boxes[i].ymin+150);

            tsinghua::dias::fusion::OBSFObs3d obs_3d;
            obs_3d._header._seq = _objects.header.seq;
            obs_3d._header._stamp._sec = sec;
            obs_3d._header._stamp._nsec = nsec;
            // obs_3d._header._frame_id = _objects.header_.frame_id;
            obs_3d._sensor_header = _objects.header;

            obs_3d._obs_id = 1111;

            obs_3d.flag_repeat = 0;
            
            obs_3d._sensor_type = 7;
            point._z = 0;
            obs_3d.rangex_rms = 100.00f;
            obs_3d.speedx_rms = 100.00f;
            obs_3d.rangey_rms = 100.00f;
            obs_3d.speedy_rms = 100.00f;
            obs_3d.rangez_rms = 100.00f;
            obs_3d.speedz_rms = 100.00f;


            //0x0:point, 0x1:car, 0x2:truck, 0x3:pedestrian, 0x4:motorcycle, 0x5:bicycle, 0x6:wide, 0x7:reserved  0x8:dog
            if (_objects.bounding_boxes[i].label == 15)
            {
                obs_3d._object_type = 0x3;
            }
            else if (_objects.bounding_boxes[i].label == 2)
            {
                obs_3d._object_type = 0x5;
            }
            else if (_objects.bounding_boxes[i].label == 7)
            {
                obs_3d._object_type = 0x1;
            }
            else
            {
                obs_3d._object_type = 0x7;
            }
            obs_3d._v_center = (resize_xmax + resize_xmin) / 2; 
            obs_3d._u_center = (resize_ymax + resize_ymin) / 2;
            obs_3d._width_pixel = resize_xmax - resize_xmin;
            obs_3d._height_pixel = resize_ymax - resize_ymin;

            Eigen::Matrix<float, 3, 3> HH;
            HH << -0.11899, -0.026076, 0.28690 * point._z - 0.8252,
                -0.22384, 0.24574, 0.045906 * point._z - 0.5648,
                -0.00059703, -0.000050721, 0.00010254 * point._z - 0.0016;

            Eigen::Matrix<float, 3, 3> HH_1 = HH.inverse();
            double bottom_u = resize_ymax;
            double bottom_v = (resize_xmax + resize_xmin) / 2;
            float xy = HH_1(2, 0) * bottom_u + HH_1(2, 1) * bottom_v + HH_1(2, 2);
            point._x = (HH_1(0, 0) * bottom_u + HH_1(0, 1) * bottom_v + HH_1(0, 2)) / xy + LRR_F_pos_x;
            point._y = (HH_1(1, 0) * bottom_u + HH_1(1, 1) * bottom_v + HH_1(1, 2)) / xy + LRR_F_pos_y;
            point._z = 0.66f;
            obs_3d._velocity._x = 0;
            obs_3d._velocity._y = 0;
            obs_3d._velocity._z = 0;    
            obs_3d._obs_position = point;
            obs_3d_obstacles.push_back(obs_3d);
        }
        
        if (obs_3d_obstacles.size() != 0)
        {
            obs_3d_frame.set_obstacles(obs_3d_obstacles);
            _Engine.process_camera_frame(obs_3d_frame);
            publish_final_fusion();
        }
    }

     void publish_final_fusion()
     {
        std::vector<tsinghua::dias::fusion::OBSFUncertaintyTrack> _obs_track;
        _obs_track = _Engine.get_tracks();
        // ICV_LOG_INFO<<"track size: "<<_obs_track.size();
        if (_obs_track.size()>0)
        {
            TrackArray track_array;
            track tracks;
            track_array.header.stamp = _obs_track[0].get_obs_radar()->_sensor_header.stamp;
            for (size_t i = 0; i < _obs_track.size(); i++)
            {
                tracks.id = _obs_track[i].get_obs_radar()->_obs_id;
                tracks.flag_repeat = _obs_track[i].get_obs_radar()->flag_repeat;
                tracks.object_type = _obs_track[i].get_obs_radar()->_object_type;
                // tracks.obs_time_stamp = _obs_track[i].get_obs_radar()->_header._stamp.to_second();
                tracks.course_angle_rad = _obs_track[i].get_obs_radar()->_course_angle_rad;
                tracks.obs_position.x = _obs_track[i].get_obs_radar()->_obs_position._x;
                tracks.obs_position.y = _obs_track[i].get_obs_radar()->_obs_position._y;
                tracks.obs_position.z = _obs_track[i].get_obs_radar()->_obs_position._z;
                tracks.velocity.x = _obs_track[i].get_obs_radar()->_velocity._x;
                tracks.velocity.y = _obs_track[i].get_obs_radar()->_velocity._y;
                tracks.velocity.z = _obs_track[i].get_obs_radar()->_velocity._z;
                tracks.object_box_center.x = _obs_track[i].get_obs_radar()->object_box_center.pose.position.x; 
                tracks.object_box_center.y = _obs_track[i].get_obs_radar()->object_box_center.pose.position.y;
                tracks.object_box_center.z = _obs_track[i].get_obs_radar()->object_box_center.pose.position.z;
                tracks.object_box_size.x = _obs_track[i].get_obs_radar()->object_box_size.x;
                tracks.object_box_size.y = _obs_track[i].get_obs_radar()->object_box_size.y;
                tracks.object_box_size.z = _obs_track[i].get_obs_radar()->object_box_size.z;
                tracks.contour_points.clear();
                for(size_t j=0; j<_obs_track[i].get_obs_radar()->contour_points.size(); j++){
                    Point contour_point;
                    contour_point.x = _obs_track[i].get_obs_radar()->contour_points[j].x;
                    contour_point.y = _obs_track[i].get_obs_radar()->contour_points[j].y;
                    contour_point.z = _obs_track[i].get_obs_radar()->contour_points[j].z;
                    tracks.contour_points.push_back(contour_point);
                }
                track_array.tracks.push_back(tracks);
            } 
            track_array.self_velocity.x = glb_twist_linear_x*cosf(course_angle_rad)+glb_twist_linear_y*sinf(course_angle_rad);
            track_array.self_velocity.y = glb_twist_linear_y*cosf(course_angle_rad)-glb_twist_linear_x*sinf(course_angle_rad);
            track_array.self_velocity.z = 0;  
            track_array.gps_route.x = route_x; 
            track_array.gps_route.y = route_y;
            track_array.gps_route.z = 0;

            // ICV_LOG_INFO<<"track array"<<track_array.tracks.size();
            fusion_output->setoutvalue(track_array);
            Send_Out(fusion_output,0);
        }
    }


    virtual void Execute(std::vector<icvNodeInput*>& inputPorts, std::vector<icvNodeOutput*>& outputPorts) override
    {
        timesync_->Lock();
        
        // std::cout<<"enter execute------"<<std::endl;
        // lrr1_data=read_Input<icvlrr>(0) ;
        // std::cout<<"enter execute------1"<<std::endl;
        lrr2_data=read_Input<icvlrr>(1) ;
        // ICV_LOG_INFO<<"received front radar num: "<<lrr2_data.targetnum_;
        // std::cout<<"enter execute------2"<<std::endl;
        // srr1_data=read_Input<icvsrr>(2) ;
        // std::cout<<"enter execute------3"<<std::endl;
        // srr2_data=read_Input<icvsrr>(3) ;
        // std::cout<<"enter execute------4"<<std::endl;
        srr3_data=read_Input<icvsrr>(4) ;
        // ICV_LOG_INFO<<"received side radar3 num: "<<srr3_data.targetnum_;
        // std::cout<<"enter execute------5"<<std::endl;
        srr4_data=read_Input<icvsrr>(5) ;
        // ICV_LOG_INFO<<"received side radar4 num: "<<srr4_data.targetnum_;
        // std::cout<<"enter execute------6"<<std::endl;

        imu_data=read_Input<icvImu>(6);
        // std::cout<<"enter execute------7"<<std::endl;
	    gps_data=read_Input<icvTwistWithCovarianceStamped>(7) ;
        // std::cout<<"enter execute------8"<<std::endl;
        imu_fix = read_Input<icvNavSatFix>(8);

        cam_data=read_Input<icvbbox2d>(9) ;

        lane_data = read_Input<icvlane>(10);
        // std::cout<<"enter execute------0"<<std::endl;
        // TODO add the definition of three functions of rear radar.
        // obs_callback_local_rear_radar_online(lrr1_data);
        //  
        //  
         if (is_not_empty(1))
         {
            obs_callback_local_front_radar_online(lrr2_data);
         }
        //  if (is_not_empty(4))
        //  {
        //     obs_callback_local_right_front_radar_online(srr3_data);
        //  }
        //  if (is_not_empty(5))
        //  {
        //     obs_callback_local_left_front_radar_online(srr4_data);
        //  }
         if (is_not_empty(7))
         {
            obs_callback_gps_vel_online(gps_data);
         }
         if (is_not_empty(6))
         {
            obs_callback_imu_data_online(imu_data);
         }

         if (is_not_empty(8))
         {
            obs_callback_gps_fix_online(imu_fix);
         }
         if (is_not_empty(9))
        {
            obs_callback_camera_bbox2d_online(cam_data);
            fusion_visual->setoutvalue(_Engine.img_lane);
            Send_Out(fusion_visual,1);
        }
        if (is_not_empty(10))
        {
            obs_callback_lane_data_online(lane_data);
        }

        // cv::Mat mFrame ;
        // mFrame = read_Input<icv::data::icvCvMatData>(11);
        // if (is_not_empty(11))
        // {
        //     cv::Mat img_resize ;
        //     cv::Rect rect(128,150,512,256);
        //     img_resize = mFrame(rect);
        //     cv::imshow("Source Image",img_resize);
        //     cv::waitKey(10);
        // }

    }


private:
    void InnerLoop()
    {
        while(true) 
        {
            timesync_->Release();
            usleep(ICV_SLEEP_CYCLE);
        }
    }
    icvSemaphore* timesync_;
    // tsinghua::dias::fusion::ObsfVisual _visualizer;
    tsinghua::dias::fusion::OBSFUncertaintyFusionEngine _Engine;
    int control_mode;
    Eigen::Affine3d transform; // cloud to radar transform
    float lidar_1 = 100;
    float lidar_2 = -100;
    float glb_twist_linear_x = 0;
    float glb_twist_linear_y = 0;
    float glb_twist_linear_z = 0;
    float course_angle_rad = 0;
    float route_x = 0;
    float route_y = 0;
    double temp_lat = 0;
    double temp_lon = 0;
    int flag_gps_frist_frame = true;
    float glb_rviz_route_x = 0;
    float glb_rviz_route_y = 0;
    uint32 lidar_time = -4;
    uint32 front_radar_time = -4;
    uint32 srr_radar_time = -4;
    uint32 now_time = -4;

    Imu imu_data ;
    TwistWithCovarianceStamped gps_data;
    NavSatFix imu_fix ;
    RadarLongOut lrr1_data ;
    RadarLongOut lrr2_data ;
    RadarSideOut srr1_data ;
    RadarSideOut srr2_data ;
    RadarSideOut srr3_data ;
    RadarSideOut srr4_data ;
    BoundingBoxes2d cam_data ;
    ld_Frame lane_data;
    
    icvfusion *fusion_output;
    icv::data::icvCvMatData* fusion_visual;
};

ICV_REGISTER_FUNCTION(SensorFusion);

#endif
