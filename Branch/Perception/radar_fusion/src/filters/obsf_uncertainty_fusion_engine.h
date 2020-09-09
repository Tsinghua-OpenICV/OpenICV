// @brief: fusion engine

// objective: processing the fusion step. From raw observation to association, state update
//Author: Chunlei YU. Contact me at: yuchunlei@mail.tsinghua.edu.cn


#ifndef OBSF_UNCERTAINTY_FUSION_ENGINE_H
#define OBSF_UNCERTAINTY_FUSION_ENGINE_H
#include <algorithm>
#include <iostream>
#include <vector>
#include <mutex>
#include <iterator>

#include "../objects/obsf_fusion_track.h"
#include "../objects/obsf_obs_frame.h"

#include "../objects/obsf_define.h"

#include <ros/ros.h>
#include <ros/time.h>

#include <visualization_msgs/MarkerArray.h>




namespace tsinghua {
namespace dias {
namespace fusion{
/*定义常量*/

float lidar_right_slope= -0.6;
float lidar_left_slope= 0.9;
float lidar_x_max_dist= 65;
float B_lidar_right_slope= 0.8;
float B_lidar_left_slope= -0.8;
float B_lidar_x_max_dist= -65;

float SRR_FR_ANGLE= 33.0*PI/180;   //wangwei
float SRR_FL_ANGLE= -27.0*PI/180;
float SRR_BR_ANGLE= 145.0*PI/180;
float SRR_BL_ANGLE= 215.0*PI/180;
float LRR_F_ANGLE= 2.0*PI/180;
float LRR_B_ANGLE= 180.0*PI/180;

float LRR_F_pos_x= 3.7;
float LRR_F_pos_y= 0.0;
float LRR_B_pos_x= -1.03;
float LRR_B_pos_y= 0.2;

float SRR_FR_pos_x= 3.68;
float SRR_FR_pos_y= -0.68;
float SRR_FL_pos_x= 3.65;
float SRR_FL_pos_y= 0.7;

float SRR_BL_pos_x= -0.94;
float SRR_BL_pos_y= -0.68;
float SRR_BR_pos_x= -0.91;
float SRR_BR_pos_y= 0.72;
float PUBLISHER_SENSOR_TYPE= 0;
float LIDAR_B_ANGLE = 180;

int f_control_mode = 255; //0 unlimited, 1 limited area to create and delete
int b_control_mode = 255; //0 unlimited, 1 limited area to create and delete
class OBSFUncertaintyFusionEngine{
public:
    typedef std::shared_ptr<OBSFObs3d> Obs3dPtr;

    OBSFUncertaintyFusionEngine(){
        init_id_arr();
    }
    ~OBSFUncertaintyFusionEngine(){}

    void init_param(float a,float b,float c,\
      float d,float e,float f,float g ,float h ,\
      float i, float j,float k,float l,float m,float n,\
      float o,float p,float q,float r,float s,float t,\
      float u,float v,float w,float x,float y,float z){ //wangwei
        lidar_right_slope = a;
        lidar_left_slope = b;
        lidar_x_max_dist = c;
        B_lidar_right_slope = d;
        B_lidar_left_slope =e;
        B_lidar_x_max_dist =f;
        SRR_FR_ANGLE= g*PI/180;   
        SRR_FL_ANGLE= h*PI/180;
        SRR_BR_ANGLE= i*PI/180;
        SRR_BL_ANGLE= j*PI/180;
        LRR_F_ANGLE= k*PI/180;
        LRR_B_ANGLE= l*PI/180;
        LRR_F_pos_x= m;
        LRR_F_pos_y= n;
        LRR_B_pos_x= o;
        LRR_B_pos_y= p;
        SRR_FR_pos_x= q;
        SRR_FR_pos_y= r;
        SRR_FL_pos_x= s;
        SRR_FL_pos_y= t;
        SRR_BL_pos_x= u;
        SRR_BL_pos_y= v;
        SRR_BR_pos_x= w;
        SRR_BR_pos_y= x;
        PUBLISHER_SENSOR_TYPE= y;
        LIDAR_B_ANGLE= z*PI/180;
    }

    void set_control_mode(int i, int j){
        sensor_type_f_or_b = j;
        if(j == 0){
            f_control_mode = i;
            
        }else{
            b_control_mode = i;
        }
        // std::cout << "f_control_mode: " << f_control_mode << std::endl;
        // std::cout << "b_control_mode: " << b_control_mode << std::endl;
        // std::cout << "sensor_type_f_or_b: " << sensor_type_f_or_b << std::endl;
    }
    void process_lidar_frame(
        const tsinghua::dias::fusion::OBSFObs3dFrame &obs_array){
        _lidar_obs_array_mutex.lock();
        _lidar_obs_array = obs_array;
        //std::cerr << "Before lidar fusion" << std::endl;
        // std::cout << "_lidar_obs_array.get_obstacles().size()"
        //           << _lidar_obs_array.get_obstacles().size()
        //           << std::endl;
        fusion(_lidar_obs_array, tsinghua::dias::fusion::LIDAR);
        _lidar_obs_array_mutex.unlock();
    }
    void process_camera_frame(
        const tsinghua::dias::fusion::OBSFObs3dFrame &obs_array){
        _came_obs_array_mutex.lock();
        //std::cerr << "Before camera fusion" << std::endl;
        _camera_obs_array = obs_array;
        // std::cout << "_camera_obs_array.get_obstacles().size()"
        //           << _camera_obs_array.get_obstacles().size()
        //           << std::endl;
        fusion(_camera_obs_array, tsinghua::dias::fusion::CAMERA3D);
        _came_obs_array_mutex.unlock();
    }

    
    void process_radar_frame(
        const tsinghua::dias::fusion::OBSFObs3dFrame &obs_array){
        
        _radar_obs_array_mutex.lock();
        _radar_obs_array = obs_array;
        fusion(_radar_obs_array, tsinghua::dias::fusion::RADAR);
        _radar_obs_array_mutex.unlock();
    }
    //void process_radar_frame();

    template <typename OBSType>
    void fusion(
        tsinghua::dias::fusion::OBSFObsFrame<OBSType> &obs_array,
        const tsinghua::dias::fusion::SensorType &sensor_type){
        _fusion_mutex.lock();
        std::vector<std::pair<int, int> > assignment;
        std::vector<int> unassigned_track;
        std::vector<int> unassigned_obs;

        //std::cout << "f_control_mode: " << f_control_mode << std::endl;

        // ROS_INFO("start fusion.............");

        //       split_lidar_radar();
        // predict_all_track_states(obs_array.get_header());

        // ROS_INFO("predict_all_track_states");

        // ROS_INFO("assign_track_obs_id_match");
        assign_track_obs_id_match(obs_array, sensor_type, assignment,
            unassigned_track, unassigned_obs);
        // assign_track_obs_id_match_lidar_only(obs_array, sensor_type, assignment,
        //     unassigned_track, unassigned_obs);

        // ROS_INFO("assign_track_obs_property_match");
        assign_track_obs_property_match(obs_array, sensor_type, assignment,
            unassigned_track, unassigned_obs);
        
        // std::cout << "assignment.size()" << assignment.size() << std::endl;
        //  for(int i=0;i<obs_array.get_obstacles().size();i++){
        //      ROS_INFO("_obs_array_Id_id:%d",obs_array.get_obstacles()[i]._obs_id);
        //      ROS_INFO("_obs_position._x:%lf",obs_array.get_obstacles()[i]._obs_position._x);
        //      ROS_INFO("_obs_position._y:%lf",obs_array.get_obstacles()[i]._obs_position._y);
        //      ROS_INFO("_obs_position._z:%lf",obs_array.get_obstacles()[i]._obs_position._z);

        //  }
        // ROS_INFO("update_assigned_track");
        update_assigned_track(obs_array, sensor_type, assignment);

        // ROS_INFO("update_unassigned_track");
        update_unassigned_track(obs_array, sensor_type, unassigned_track);

        // ROS_INFO("delete_lost_track");
        delete_lost_track();

        // ROS_INFO("create_new_track");
        
        create_new_track(obs_array, sensor_type, unassigned_obs);

        // ROS_INFO("fusion finished");

        //        delete_track_with_null_lidar();   

        //idl::car::map2d23d::Util::print_fusion_result(_obs_track);

        _fusion_mutex.unlock();
    }

    void predict_all_track_states(const tsinghua::dias::fusion::OBSFHeader &header){//double observation_time
        for (size_t i = 0; i < _obs_track.size(); i++){
            _obs_track[i].prediction(header);
        }
    }

    // just delete radar obstacles
    void split_lidar_radar(){
        //int track_num = 0;
        for (size_t i = 0; i < _obs_track.size(); i++){
            if (_obs_track[i].get_obs_lidar() != NULL &&
                _obs_track[i].get_obs_radar() != NULL){
                double diff_x = _obs_track[i].get_obs_lidar()->_obs_position._x -
                                _obs_track[i].get_obs_radar()->_obs_position._x;
                double diff_y = _obs_track[i].get_obs_lidar()->_obs_position._y -
                                _obs_track[i].get_obs_radar()->_obs_position._y;
                double distance = sqrt(pow(diff_x, 2) + pow(diff_y, 2));
                if (distance > tsinghua::dias::fusion::FUSION_THRES * 1.0) {
                    _obs_track[i].set_obs_radar(NULL);
                }
            }
        }
        //std::cout << "_obs_track.size()" << _obs_track.size() << std::endl;
        //_obs_track.resize(track_num);
        //std::cout << "_obs_track.size()" << _obs_track.size() << std::endl;
    }

    void delete_track_with_null_lidar(){
        int track_num = 0;
        for (size_t i = 0; i < _obs_track.size(); i++){

            double local_x = _obs_track[i].get_obs()->_obs_local_position._x;
            double local_y = _obs_track[i].get_obs()->_obs_local_position._y;

            double local_range = sqrt(local_x * local_x + local_y * local_y);

            if((local_range <= 40) && (_obs_track[i].get_obs_lidar() == NULL))
                continue;

            _obs_track[track_num++] = _obs_track[i];
        }
        //std::cout << "_obs_track.size()" << _obs_track.size() << std::endl;
        _obs_track.resize(track_num);
        //std::cout << "_obs_track.size()" << _obs_track.size() << std::endl;
    }

    template <typename OBSType>
    void create_track_area_partition_function(int t_control_mode,int t_sensor_type, 
        tsinghua::dias::fusion::OBSFObsFrame<OBSType> &obs_array,
        const tsinghua::dias::fusion::SensorType &sensor_type,
        std::vector<int>& unassigned_obs,
        float lrr_mid_pos_x, float lrr_mid_pos_y,
        float lidar_rght_slope, float lidar_lft_slope){
        /*radar detection zone limit
        Radar generated track limit, all the laser radar is reserved, the forward radar is retained 65 meters, 
        the right front radar is retained more than -0.6 slope, the left front radar is more than -0.6 slope.*/
        if (t_control_mode == -1 || t_sensor_type!=7){
            for (int i = 0; i < unassigned_obs.size(); i++){
                obs_array.get_obstacles()[unassigned_obs[i]]._obs_id=get_id();
                _obs_track.push_back(tsinghua::dias::fusion::OBSFUncertaintyTrack(
                    obs_array.get_obstacles()[unassigned_obs[i]], sensor_type));
            }
        }else if(t_control_mode == 111){  //lidar radar all died
            for (int i = 0; i < unassigned_obs.size(); i++){
                obs_array.get_obstacles()[unassigned_obs[i]]._obs_id=get_id();
                _obs_track.push_back(tsinghua::dias::fusion::OBSFUncertaintyTrack(
                    obs_array.get_obstacles()[unassigned_obs[i]], sensor_type));
            }
        }else{
            if(t_sensor_type==6 || t_sensor_type==8){
                for (int i = 0; i < unassigned_obs.size(); i++){ 
                    obs_array.get_obstacles()[unassigned_obs[i]]._obs_id=get_id();        
                    _obs_track.push_back(tsinghua::dias::fusion::OBSFUncertaintyTrack(
                        obs_array.get_obstacles()[unassigned_obs[i]], sensor_type));          
                }
            }else if(t_sensor_type==1 || t_sensor_type==5){
                for (int i = 0; i < unassigned_obs.size(); i++){
                    if (((obs_array.get_obstacles()[unassigned_obs[i]]._obs_position._x - lrr_mid_pos_x) / (obs_array.get_obstacles()[unassigned_obs[i]]._obs_position._y ) > lidar_rght_slope) && (obs_array.get_obstacles()[unassigned_obs[i]]._obs_position._y < 0))
                    {
                        obs_array.get_obstacles()[unassigned_obs[i]]._obs_id=get_id();
                        _obs_track.push_back(tsinghua::dias::fusion::OBSFUncertaintyTrack(
                            obs_array.get_obstacles()[unassigned_obs[i]], sensor_type));          
                    }
                }
            }else if(t_sensor_type==2 || t_sensor_type==4){
                for (int i = 0; i < unassigned_obs.size(); i++){
                    if (((obs_array.get_obstacles()[unassigned_obs[i]]._obs_position._x - lrr_mid_pos_x) / (obs_array.get_obstacles()[unassigned_obs[i]]._obs_position._y ) < lidar_lft_slope) && (obs_array.get_obstacles()[unassigned_obs[i]]._obs_position._y > 0))
                    {
                        obs_array.get_obstacles()[unassigned_obs[i]]._obs_id=get_id();
                        _obs_track.push_back(tsinghua::dias::fusion::OBSFUncertaintyTrack(
                            obs_array.get_obstacles()[unassigned_obs[i]], sensor_type));          
                    }
                }
            }else if(t_sensor_type==0 || t_sensor_type==3){
                for (int i = 0; i < unassigned_obs.size(); i++){         
                    // if(obs_array.get_obstacles()[unassigned_obs[i]]._obs_position._x > lidar_x_max_dist){  
                        // std::cout << "obs_array.get_obstacles()[unassigned_obs[i]]._obs_position._x" << obs_array.get_obstacles()[unassigned_obs[i]]._obs_position._x << std::endl;       
                        obs_array.get_obstacles()[unassigned_obs[i]]._obs_id=get_id();
                        _obs_track.push_back(tsinghua::dias::fusion::OBSFUncertaintyTrack(
                            obs_array.get_obstacles()[unassigned_obs[i]], sensor_type));          
                    // }
                }
            }
        }
    }

    template <typename OBSType>
    void create_new_track(
        tsinghua::dias::fusion::OBSFObsFrame<OBSType> &obs_array,
        const tsinghua::dias::fusion::SensorType &sensor_type,
        std::vector<int>& unassigned_obs
        ){
        
        if(obs_array.get_obstacles()[0]._sensor_type != 7){
            for (int i = 0; i < unassigned_obs.size(); i++){                     
                    obs_array.get_obstacles()[unassigned_obs[i]].P.setZero();
                    obs_array.get_obstacles()[unassigned_obs[i]].P(0,0) = obs_array.get_obstacles()[unassigned_obs[i]].rangex_rms;
                    obs_array.get_obstacles()[unassigned_obs[i]].P(1,1) = obs_array.get_obstacles()[unassigned_obs[i]].rangey_rms;
                    obs_array.get_obstacles()[unassigned_obs[i]].P(2,2) = obs_array.get_obstacles()[unassigned_obs[i]].speedy_rms;
                    obs_array.get_obstacles()[unassigned_obs[i]].P(3,3) = obs_array.get_obstacles()[unassigned_obs[i]].speedy_rms;
                    obs_array.get_obstacles()[unassigned_obs[i]].P(4,4) = obs_array.get_obstacles()[unassigned_obs[i]].ori_rms;
                    obs_array.get_obstacles()[unassigned_obs[i]].P(5,5) = obs_array.get_obstacles()[unassigned_obs[i]].oriRate_rms;

                    tsinghua::dias::fusion::hist_track_obs temp_hist_obs;
                    tsinghua::dias::fusion::hist_track_obs temp_hist_track;

                    temp_hist_track._self_velocity = obs_array.get_obstacles()[unassigned_obs[i]]._self_velocity ;
                    temp_hist_track._header = obs_array.get_obstacles()[unassigned_obs[i]]._header._stamp.to_second();
                    temp_hist_track._obs_position._x = obs_array.get_obstacles()[unassigned_obs[i]]._obs_position._x;
                    temp_hist_track._obs_position._y = obs_array.get_obstacles()[unassigned_obs[i]]._obs_position._y;
                    temp_hist_track._velocity._x = obs_array.get_obstacles()[unassigned_obs[i]]._velocity._x;
                    temp_hist_track._velocity._y = obs_array.get_obstacles()[unassigned_obs[i]]._velocity._y;
                    temp_hist_track.P = obs_array.get_obstacles()[unassigned_obs[i]].P;
                    temp_hist_track.rangex_rms = obs_array.get_obstacles()[unassigned_obs[i]].rangex_rms;
                    temp_hist_track.speedx_rms = obs_array.get_obstacles()[unassigned_obs[i]].speedx_rms;
                    temp_hist_track.rangey_rms = obs_array.get_obstacles()[unassigned_obs[i]].rangey_rms;
                    temp_hist_track.speedy_rms = obs_array.get_obstacles()[unassigned_obs[i]].speedy_rms;
                    temp_hist_track.ori_rms = obs_array.get_obstacles()[unassigned_obs[i]].ori_rms;
                    temp_hist_track.oriRate_rms = obs_array.get_obstacles()[unassigned_obs[i]].oriRate_rms;

                    temp_hist_track._sensor_type = obs_array.get_obstacles()[unassigned_obs[i]]._sensor_type;
                    obs_array.get_obstacles()[unassigned_obs[i]]._hist_states.push_front(temp_hist_track); // store the history in hist_states;
                    
                    temp_hist_obs = temp_hist_track;
                    obs_array.get_obstacles()[unassigned_obs[i]]._hist_obs.push_front(temp_hist_obs); // store the history in hist_obs;
            }

            for (int i = 0; i < unassigned_obs.size(); i++){
                obs_array.get_obstacles()[unassigned_obs[i]]._obs_id=get_id();
                obs_array.get_obstacles()[unassigned_obs[i]]._tracks_lane = lane_interior_point_judge(obs_array.get_obstacles()[unassigned_obs[i]]._obs_position);
                double lane_timp_flag = ros::Time::now().toSec() - _lane_time_stamp;
                if(lane_timp_flag > 1){
                    _lane_sequence.clear();
                    _AutoDrive_lane.clear();
                }
                obs_array.get_obstacles()[unassigned_obs[i]]._ctype_update = 0;
                _obs_track.push_back(tsinghua::dias::fusion::OBSFUncertaintyTrack(
                        obs_array.get_obstacles()[unassigned_obs[i]], sensor_type));
            }
        }
        // int sensor_type_num = obs_array.get_obstacles()[0]._sensor_type;
        // float lrr_mid_x, lrr_mid_y, ldr_right_slope, ldr_left_slope;
        // int control_mode;
        // if(sensor_type_f_or_b == 0){
        //     lrr_mid_x = LRR_F_pos_x;
        //     lrr_mid_y = LRR_F_pos_y;
        //     ldr_right_slope = lidar_right_slope;
        //     ldr_left_slope = lidar_left_slope;
        //     control_mode = f_control_mode;
        // }else{
        //     lrr_mid_x = LRR_B_pos_x;
        //     lrr_mid_y = LRR_B_pos_y;
        //     ldr_right_slope = B_lidar_left_slope;
        //     ldr_left_slope = B_lidar_right_slope;
        //     control_mode = b_control_mode;
        // }
        // create_track_area_partition_function(control_mode, sensor_type_num, obs_array,sensor_type,
        //                         unassigned_obs, lrr_mid_x, lrr_mid_y, ldr_right_slope, ldr_left_slope);

        /*Generating new tracks based on distance between tracks.
        Determination conditions: the number of iterations / distance from the car.*/
        double max_dist = tsinghua::dias::fusion::FUSION_THRES*2;
        float track_distance;
        float track_angle_diff;
        float vel_diff;
        std::vector<bool> flag_track_distance;
        flag_track_distance.resize(_obs_track.size());
        for (int i = 0; i < _obs_track.size(); ++i) {flag_track_distance[i] = 0;}

        for (int i = 0; i < _obs_track.size(); ++i){
            //  if(i<3)    
            //         std::cout << "[--]sensorT:"<<_obs_track[i].get_obs_radar()->_sensor_type<< " ID:"<<_obs_track[i].get_obs_radar()->_obs_id << " x:"<<lidar_box.x<<std::endl;
            for (int j = i + 1; j < _obs_track.size(); ++j){
                track_distance = 
                    sqrt(pow((_obs_track[i].get_obs_radar()->_obs_position._x - _obs_track[j].get_obs_radar()->_obs_position._x), 2) 
                    + pow((_obs_track[i].get_obs_radar()->_obs_position._y - _obs_track[j].get_obs_radar()->_obs_position._y), 2));
                vel_diff = 
                    sqrt(pow((_obs_track[i].get_obs_radar()->_velocity._x - _obs_track[j].get_obs_radar()->_velocity._x), 2) 
                    + pow((_obs_track[i].get_obs_radar()->_velocity._y - _obs_track[j].get_obs_radar()->_velocity._y), 2));
                
                if (track_distance < max_dist && vel_diff < 2){
                    if (_obs_track[i].get_obs_radar()->flag_repeat > 20 && _obs_track[j].get_obs_radar()->flag_repeat > 20){
                        float dist1_to_car = sqrt(pow(_obs_track[i].get_obs_radar()->_obs_position._x, 2) + pow(_obs_track[i].get_obs_radar()->_obs_position._y, 2));
                        float dist2_to_car = sqrt(pow(_obs_track[j].get_obs_radar()->_obs_position._x, 2) + pow(_obs_track[j].get_obs_radar()->_obs_position._y, 2));
                        float c = track_distance;
                        float a, b;
                        if(dist1_to_car < dist2_to_car){
                            a=dist1_to_car;
                            b=dist2_to_car;
                        }
                        else{
                            a = dist2_to_car;
                            b = dist1_to_car;
                        }
                        track_angle_diff=acosf((a*a+c*c-b*b)/(2*a*c));
                        if(track_angle_diff > 3.14/180*150 || track_distance<max_dist/4){
                            if(dist1_to_car < dist2_to_car){
                                flag_track_distance[j] = 1;
                            }
                            else{
                                flag_track_distance[i] = 1;
                            }
                        }
                    }
                    else{
                        if (_obs_track[i].get_obs_radar()->flag_repeat < _obs_track[j].get_obs_radar()->flag_repeat){
                            flag_track_distance[i] = 1;
                        }
                        else{
                            flag_track_distance[j] = 1;
                        }
                    }
                }            
            }
        }

        //delete
        int track_num = 0;
        for (size_t i = 0; i < _obs_track.size(); i++){
            // _obs_track[track_num++] = _obs_track[i];
            if (flag_track_distance[i] == 0){
                _obs_track[track_num++] = _obs_track[i]; //assign pair will get useless after this. jxy 0522
            }else{
                check_id(_obs_track[i].get_obs_radar()->_obs_id);
            }
        }
        //std::cout << "_obs_track.size()" << _obs_track.size() << std::endl;
        _obs_track.resize(track_num);
        obs_array_sensor_type = obs_array.get_obstacles()[0]._sensor_type;
        // std::cout << "============_obs_track.size()============" << _obs_track.size()+id_arr.size() << std::endl;
    }

    void delete_lost_track(){
        int track_num = 0;

        for (size_t i = 0; i < _obs_track.size(); i++){
            if (_obs_track[i].get_obs_lidar() != NULL
                || _obs_track[i].get_obs_camera() != NULL
                || _obs_track[i].get_obs_radar() != NULL){
                _obs_track[track_num++] = _obs_track[i]; //assign pair will get useless after this. jxy 0522
            }
        }
        //std::cout << "_obs_track.size()" << _obs_track.size() << std::endl;
        _obs_track.resize(track_num);
        //std::cout << "_obs_track.size()" << _obs_track.size() << std::endl;
    }

    template <typename OBSType>
    void store_history_tracks_obs(int i, OBSType &ob
        ){        
            tsinghua::dias::fusion::hist_track_obs temp_hist_track;
            temp_hist_track._self_velocity = _obs_track[i].get_obs_radar()->_self_velocity ;
            temp_hist_track._header = _obs_track[i].get_obs_radar()->_header._stamp.to_second();
            temp_hist_track._obs_position._x = _obs_track[i].get_obs_radar()->_obs_position._x;
            temp_hist_track._obs_position._y = _obs_track[i].get_obs_radar()->_obs_position._y;
            temp_hist_track._velocity._x = _obs_track[i].get_obs_radar()->_velocity._x;
            temp_hist_track._velocity._y = _obs_track[i].get_obs_radar()->_velocity._y;
            temp_hist_track.P = _obs_track[i].get_obs_radar()->P;
            temp_hist_track._sensor_type = _obs_track[i].get_obs_radar()->_sensor_type;
            _obs_track[i].get_obs_radar()->_hist_states.push_front(temp_hist_track);   // store the history in hist_states;
            if(_obs_track[i].get_obs_radar()->_hist_states.size() > 10){  _obs_track[i].get_obs_radar()->_hist_states.pop_back();  }
            // std::cout<<"_obs_id:"<<_obs_track[i].get_obs_radar()->_obs_id <<"  track_raw._hist_states.size():"<<_obs_track[i].get_obs_radar()->_hist_states.size()<<std::endl;
            tsinghua::dias::fusion::hist_track_obs temp_hist_obs;
            temp_hist_obs._header = ob._header._stamp.to_second();
            temp_hist_obs._obs_position._x = ob._obs_position._x;
            temp_hist_obs._obs_position._y = ob._obs_position._y;
            temp_hist_obs._velocity._x = ob._velocity._x;
            temp_hist_obs._velocity._y = ob._velocity._y;
            temp_hist_obs.rangex_rms = ob.rangex_rms;                
            temp_hist_obs.speedx_rms = ob.speedx_rms;                
            temp_hist_obs.rangey_rms = ob.rangey_rms;
            temp_hist_obs.speedy_rms = ob.speedy_rms;
            temp_hist_obs.ori_rms = ob.ori_rms;
            temp_hist_obs.oriRate_rms = ob.oriRate_rms;
            _obs_track[i].get_obs_radar()->_hist_obs.push_front(temp_hist_obs); // store the history in hist_obs;
            if(_obs_track[i].get_obs_radar()->_hist_obs.size() > 10){ _obs_track[i].get_obs_radar()->_hist_obs.pop_back(); }
        }

    template <typename OBSType>
    void update_assigned_track(
        tsinghua::dias::fusion::OBSFObsFrame<OBSType> &obs_array,
        const tsinghua::dias::fusion::SensorType &sensor_type,
        const std::vector<std::pair<int, int> > &assignment
        ){
        //std::cout << "assignment.size()" << assignment.size() << std::endl;
        double time_stamp = obs_array.get_header()._stamp.to_second();

        for (size_t i = 0; i < assignment.size(); i++){
            auto _obs_array = obs_array.get_obstacles()[assignment[i].second];
            auto obs_track = _obs_track[assignment[i].first].get_obs_radar();

            obs_track->flag_repeat++;
            _obs_array.flag_repeat = obs_track->flag_repeat;
            _obs_array._obs_id = obs_track->_obs_id;
            _obs_array._ctype_update = obs_track->_ctype_update;
            _obs_array._hist_obs = obs_track->_hist_obs;
            _obs_array._hist_states = obs_track->_hist_states;
            if( _obs_array._sensor_type != 6 && _obs_array._sensor_type != 8){
                _obs_array.object_box_center = obs_track->object_box_center;
                _obs_array.object_box_size = obs_track->object_box_size;
                _obs_array.contour_points = obs_track->contour_points;
            }else{
                _obs_array._velocity = obs_track->_velocity;
            }
            if(_obs_array._sensor_type != 0 && _obs_array._sensor_type != 3){
                _obs_array._acceleration = obs_track->_acceleration;
                _obs_array._obs_theta = obs_track->_obs_theta;
            }

            if(_obs_array._sensor_type == 7){
                // _obs_array._Box3d.x = std::min(std::max(obs_track->_Box3d.x,\
                //                         _obs_array._Box3d.x),4.4);
                // _obs_array._Box3d.y = std::min(std::max(obs_track->_Box3d.y,\
                //                         _obs_array._Box3d.y),2.0);
                // _obs_array._Box3d.z = std::min(std::max(obs_track->_Box3d.z,\
                //                         _obs_array._Box3d.z),2.0);
                _obs_array._obs_position = obs_track->_obs_position;
                _obs_array._velocity = obs_track->_velocity;
                if( obs_track->_ctype_update < 4 )  {_obs_array._object_type = 0;}          
            }
            auto camera_box_size = _obs_array._Box3d;  //camera X kalman
            auto camera_obj_type = _obs_array._object_type; 
            
            if(_obs_array._sensor_type != 7){
                if(obs_track->flag_repeat>4 && fabs(_obs_array._obs_theta)<0.26 && obs_track->_object_type !=0){
                    _obs_array._Box3d.x = std::min(std::max(obs_track->_Box3d.x,\
                                        _obs_array.object_box_size.x),4.4);
                    _obs_array._Box3d.y = std::min(std::max(obs_track->_Box3d.y,\
                                        _obs_array.object_box_size.y),2.0);
                    _obs_array._Box3d.z = std::min(std::max(obs_track->_Box3d.z,\
                                        _obs_array.object_box_size.z),2.0);
                }else{
                    _obs_array._Box3d.x = _obs_array.object_box_size.x;
                    _obs_array._Box3d.y = _obs_array.object_box_size.y;
                    _obs_array._Box3d.z = _obs_array.object_box_size.z; 
                }
                _obs_array._object_type = obs_track->_object_type;    
            }
            if(_obs_array._sensor_type != 7)
                _obs_track[assignment[i].first].set_obs(*obs_track, _obs_array, sensor_type); //kalman filter
            // std::cout << "flag: " << obs_track->flag_repeat << std::endl;

            if(_obs_array._sensor_type == 7){ //camera X kalman
                camera_box_size.x = std::min(camera_box_size.x, 4.4);
                camera_box_size.y = std::min(camera_box_size.x, 2.0);
                camera_box_size.z = std::min(camera_box_size.x, 2.0);
                obs_track->_Box3d = camera_box_size;
                obs_track->_object_type = camera_obj_type;           
            }

            for (int j = 0; j < obs_track->contour_points.size(); j++){ //translate together with box
                obs_track->contour_points[j].x = obs_track->contour_points[j].x 
                - obs_track->object_box_center.pose.position.x + obs_track->_obs_position._x;
                obs_track->contour_points[j].y = obs_track->contour_points[j].y 
                - obs_track->object_box_center.pose.position.y + obs_track->_obs_position._y;
            }
            obs_track->object_box_center.pose.position.x = obs_track->_obs_position._x;
            obs_track->object_box_center.pose.position.y = obs_track->_obs_position._y;
            
            //     ROS_INFO("CALLBACK_FRONT_RADAR_point._y:%f",_obs_track[i].get_obs_radar()->_obs_position._y);
            obs_track->_tracks_lane = lane_interior_point_judge(obs_track->_obs_position);
            store_history_tracks_obs(assignment[i].first, _obs_array);
        }

        // for (size_t i = 0; i < assignment.size(); i++)
        // {
        //     std::cout << "after update: " << std::endl;
        //     std::cout << "assignment[" << i << "]: " << assignment[i].first << " " << assignment[i].second << std::endl;
        //     std::cout << "after id[" << _obs_track[assignment[i].first].get_obs_radar()->_obs_id << std::endl;
        //     std::cout << "track assignment.x " << _obs_track[assignment[i].first].get_obs_radar()->_obs_position._x << " obs_assignment.x " << obs_array.get_obstacles()[assignment[i].second]._obs_position._x << std::endl;
        //     std::cout << "track assignment.y " << _obs_track[assignment[i].first].get_obs_radar()->_obs_position._y << " obs_assignment.y " << obs_array.get_obstacles()[assignment[i].second]._obs_position._y << std::endl;
        //     std::cout << "track assignment.vx " << _obs_track[assignment[i].first].get_obs_radar()->_velocity._x << " obs_assignment.vx " << obs_array.get_obstacles()[assignment[i].second]._velocity._x << std::endl;
        //     std::cout << "track assignment.vy " << _obs_track[assignment[i].first].get_obs_radar()->_velocity._y << " obs_assignment.vy " << obs_array.get_obstacles()[assignment[i].second]._velocity._y << std::endl;
        // }
    }

    void delete_track_area_partition_function(int t_control_mode,int t_sensor_type, 
        const tsinghua::dias::fusion::SensorType &sensor_type,
        std::vector<int>& unassigned_track,
        float lidar_mid_max_x, float trk_x, float trk_y, float lidar_rght_slope, 
        float lidar_lft_slope, float trk_slope, int i){

        if(t_control_mode == -1 && t_sensor_type != 7){
            check_id(_obs_track[unassigned_track[i]].get_obs_radar()->_obs_id); 
            _obs_track[unassigned_track[i]].set_obs_radar(NULL);
        }else if(t_control_mode == 111){  //lidar radar all died
            check_id(_obs_track[unassigned_track[i]].get_obs_radar()->_obs_id); 
            _obs_track[unassigned_track[i]].set_obs_radar(NULL);
        }else{
            if (t_sensor_type == 6 || t_sensor_type == 8){
                if(t_control_mode == 0){ //front radar died, delete by lidar
                    if (trk_slope > lidar_lft_slope || trk_slope < lidar_rght_slope){
                        check_id(_obs_track[unassigned_track[i]].get_obs_radar()->_obs_id);
                        _obs_track[unassigned_track[i]].set_obs_radar(NULL);
                    }
                }
                else if(t_control_mode == 1){ //srr died, delete near by lidar
                    if (trk_x < lidar_mid_max_x){
                        check_id(_obs_track[unassigned_track[i]].get_obs_radar()->_obs_id);
                        _obs_track[unassigned_track[i]].set_obs_radar(NULL);
                    }
                }
                else{
                    if (trk_x < lidar_mid_max_x && (trk_slope > lidar_lft_slope || trk_slope < lidar_rght_slope)){
                        check_id(_obs_track[unassigned_track[i]].get_obs_radar()->_obs_id);
                        _obs_track[unassigned_track[i]].set_obs_radar(NULL);
                    }
                }
            }
            else if (t_sensor_type == 0 || t_sensor_type == 3){
                if(t_control_mode == 6){ //lidar died, delete by front radar
                    if (trk_slope > lidar_lft_slope || trk_slope < lidar_rght_slope){
                        check_id(_obs_track[unassigned_track[i]].get_obs_radar()->_obs_id);
                        _obs_track[unassigned_track[i]].set_obs_radar(NULL);
                    }
                }
                else{ //control mode is 255 or 1
                    if (trk_x > lidar_mid_max_x){
                        check_id(_obs_track[unassigned_track[i]].get_obs_radar()->_obs_id);
                        _obs_track[unassigned_track[i]].set_obs_radar(NULL);
                    }
                }
            }
            else if (t_sensor_type == 1 || t_sensor_type == 5){
                if ((trk_slope > lidar_rght_slope) && (trk_y < 0)){
                    check_id(_obs_track[unassigned_track[i]].get_obs_radar()->_obs_id);
                    _obs_track[unassigned_track[i]].set_obs_radar(NULL);
                }
            }
            else if (t_sensor_type == 2 || t_sensor_type == 4){
                if ((trk_slope < lidar_lft_slope) && (trk_y > 0)){
                    check_id(_obs_track[unassigned_track[i]].get_obs_radar()->_obs_id);
                    _obs_track[unassigned_track[i]].set_obs_radar(NULL);
                }
            }
        }
    }

    template <typename OBSType>
    void update_unassigned_track(
        tsinghua::dias::fusion::OBSFObsFrame<OBSType> &obs_array,
        const tsinghua::dias::fusion::SensorType &sensor_type,
        std::vector<int> &unassigned_track
        ){
        double time_stamp = obs_array.get_header()._stamp.to_second();
        for (size_t i = 0; i < unassigned_track.size(); i++){
            if (sensor_type == LIDAR && _obs_track[unassigned_track[i]].get_obs_lidar() != NULL){
                // double lidar_time =
                //     _obs_track[unassigned_track[i]].get_obs_lidar()->_header._stamp.to_second();
                // double time_diff = fabs(time_stamp - lidar_time);
                // if (time_diff > LIDAR_TIME_WIN){
                //     _obs_track[unassigned_track[i]].set_obs_lidar(NULL);
                // }
                _obs_track[unassigned_track[i]].set_obs_lidar(NULL);
            } else if (sensor_type == CAMERA3D &&
                _obs_track[unassigned_track[i]].get_obs_camera() != NULL){
                double camera_time =
                    _obs_track[unassigned_track[i]].get_obs_camera()->_header._stamp.to_second();
                double time_diff = fabs(time_stamp - camera_time);
                if (time_diff > CAMERA_TIME_WIN){
                    _obs_track[unassigned_track[i]].set_obs_camera(NULL);
                }
            } else if (sensor_type == RADAR &&
                _obs_track[unassigned_track[i]].get_obs_radar() != NULL) {
                double radar_time =
                    _obs_track[unassigned_track[i]].get_obs_radar()->_header._stamp.to_second();
                double time_diff = fabs(time_stamp - radar_time);
                if (time_diff > RADAR_TIME_WIN) {

                    check_id(_obs_track[unassigned_track[i]].get_obs_radar()->_obs_id); 
                    _obs_track[unassigned_track[i]].set_obs_radar(NULL);
                    // int sensor_type_num = obs_array.get_obstacles()[0]._sensor_type;
                    // int ii = i;
                    // float lrr_mid_x, lrr_mid_y, ldr_right_slope, ldr_left_slope, mid_max_x;
                    // int control_mode;
                    // if(sensor_type_f_or_b == 0){
                    //     lrr_mid_x = LRR_F_pos_x;
                    //     lrr_mid_y = LRR_F_pos_y;
                    //     mid_max_x = lidar_x_max_dist;
                    //     ldr_right_slope = lidar_right_slope;
                    //     ldr_left_slope = lidar_left_slope;
                    //     control_mode = f_control_mode;
                    // }else{
                    //     lrr_mid_x = LRR_B_pos_x;
                    //     lrr_mid_y = LRR_B_pos_y;
                    //     mid_max_x = B_lidar_x_max_dist;
                    //     ldr_right_slope = B_lidar_left_slope;  
                    //     ldr_left_slope = B_lidar_right_slope;
                    //     control_mode = b_control_mode;
                    // }
                    // float track_x = _obs_track[unassigned_track[i]].get_obs_radar()->_obs_position._x;
                    // float track_y = _obs_track[unassigned_track[i]].get_obs_radar()->_obs_position._y;
                    // float track_slope = (_obs_track[unassigned_track[i]].get_obs_radar()->_obs_position._x - lrr_mid_x) / (_obs_track[unassigned_track[i]].get_obs_radar()->_obs_position._y);
                    // delete_track_area_partition_function(control_mode, sensor_type_num, sensor_type,
                    //             unassigned_track, mid_max_x,track_x, track_y, ldr_right_slope, ldr_left_slope, track_slope, ii);                              
                }
            } else {
                //LOG(INFO) << "Unassigned" << std::endl;
                //LOG(INFO) << "SensorType:" << sensor_type << std::endl;
            }
        }
    }

    template <typename OBSType>
    void assign_track_obs_id_match(
        tsinghua::dias::fusion::OBSFObsFrame<OBSType> &obs_array,
        SensorType sensor_type,
        std::vector<std::pair<int, int> > &assignment,
        std::vector<int> &unassigned_track,
        std::vector<int> &unassigned_obs
        ){
        assignment.resize(_obs_track.size());  //problem detected: obs_track.size() shouldn't be smaller than assigned ones. why so many repeated ones? jxy 0517
        int assignment_num = 0;
        std::vector<bool> track_used(_obs_track.size(), false);
        std::vector<bool> obs_used(obs_array.get_obstacles().size(), false);
        for (size_t i = 0; i < _obs_track.size(); i++){
            std::shared_ptr<OBSType> obs;
            if (sensor_type == LIDAR){
                obs = _obs_track[i].get_obs_lidar();
            } else if (sensor_type == CAMERA3D){
                obs = _obs_track[i].get_obs_camera();
            } else if (sensor_type == RADAR){
                obs = _obs_track[i].get_obs_radar();
            } else{
                std::cout << "Unknown SensorType!" << std::endl;
            }
            if (obs == NULL){
                //std::cout << "obs == NULL" << std::endl;
                continue;
            }
        // std::cout << "track  p:\n" << obs->P << std::endl;
        // std::cout << "track  xy:\n" << obs->_obs_position._x << "  "<<obs->_obs_position._y << std::endl;
        // std::cout << "track  vv:\n" << obs->_velocity._x <<"  " << obs->_velocity._y  <<std::endl;
            // float distance_dev = 0;
            // for (size_t j = 0; j < obs_array.get_obstacles().size(); j++){
                // if (obs->_obs_id == obs_array.get_obstacles()[j]._obs_id && obs->_obs_id != 0) //there are repeated 0 id for some unknown reasons. essential to calculate distance for safety. jxy 0518
            //     {
            //         distance_dev = sqrt(pow((obs->_obs_position._x - obs_array.get_obstacles()[j]._obs_position._x), 2) + pow((obs->_obs_position._y - obs_array.get_obstacles()[j]._obs_position._y), 2));
            //         if(distance_dev <= tsinghua::dias::fusion::FUSION_THRES){
            //             assignment[assignment_num++] = std::make_pair(i, j);
            //             track_used[i] = true;
            //             obs_used[j] = true;
            //             continue;
            //         }                    
            //         //std::cout << "match id" << std::endl;

		    // //_obs_track[i].increase_tracked_times();	//test
            //     }
            // }
        }
        //std::cout << "1" << std::endl;
        //std::cout << _obs_track.size() << "assignment size: " << assignment.size() << std::endl;
        //std::cout << "assignment_num: " << assignment_num << std::endl;

        // for(int i=0;i<assignment.size();i++){
        //     std::cout << "assignment " << i << ": " << assignment[i].first << " " << assignment[i].second << " old id: " << _obs_track[assignment[i].first].get_obs_radar()->_obs_id << " new id: " << obs_array.get_obstacles()[assignment[i].second]._obs_id << std::endl;
        //     //jxy: why so many repeated zeros?
        // }

        assignment.resize(assignment_num);
        unassigned_track.resize(_obs_track.size());
        int unassigned_track_num = 0;
        for (size_t i = 0; i < track_used.size(); i++){
            if (track_used[i] == false){
                unassigned_track[unassigned_track_num++] = i;
            }
        }
        unassigned_track.resize(unassigned_track_num);
        unassigned_obs.resize(obs_array.get_obstacles().size());
        int unassigned_obs_num = 0;
        for (size_t i = 0; i < obs_used.size(); i++){
            obs_array.get_obstacles()[i]._tracks_lane = 11;
            if (obs_used[i] == false){
                unassigned_obs[unassigned_obs_num++] = i;
            }
        }
        unassigned_obs.resize(unassigned_obs_num);
        //std::cout << "assignment.size()" << assignment.size() << std::endl;
    }

    template <typename OBSType>
    void assign_track_obs_id_match_lidar_only(
        tsinghua::dias::fusion::OBSFObsFrame<OBSType> &obs_array,
        SensorType sensor_type,
        std::vector<std::pair<int, int> > &assignment,
        std::vector<int> &unassigned_track,
        std::vector<int> &unassigned_obs
        ){
        assignment.resize(_obs_track.size());
        int assignment_num = 0;
        std::vector<bool> track_used(_obs_track.size(), false);
        std::vector<bool> obs_used(obs_array.get_obstacles().size(), false);
        for (size_t i = 0; i < _obs_track.size(); i++){
            std::shared_ptr<OBSType> obs;
            if (sensor_type == LIDAR){
                obs = _obs_track[i].get_obs_lidar();
            } else if (sensor_type == CAMERA3D){
                obs = _obs_track[i].get_obs_camera();
            } else if (sensor_type == RADAR){
                obs = _obs_track[i].get_obs_radar();
            } else{
                std::cout << "Unknown SensorType!" << std::endl;
            }

            //lidar only
            if (obs == NULL || sensor_type == tsinghua::dias::fusion::RADAR){
                //std::cout << "obs == NULL" << std::endl;
                continue;
            }

            for (size_t j = 0; j < obs_array.get_obstacles().size(); j++){
                if (obs->_obs_id == obs_array.get_obstacles()[j]._obs_id){
                    assignment[assignment_num++] = std::make_pair(i, j);
                    track_used[i] = true;
                    obs_used[j] = true;
                    //std::cout << "match id" << std::endl;
                }
            }
        }
        //std::cerr << "1" << std::endl;
        assignment.resize(assignment_num);
        unassigned_track.resize(_obs_track.size());
        int unassigned_track_num = 0;
        for (size_t i = 0; i < track_used.size(); i++){
            if (track_used[i] == false){
                unassigned_track[unassigned_track_num++] = i;
            }
        }
        unassigned_track.resize(unassigned_track_num);
        //std::cerr << "2" << std::endl;
        unassigned_obs.resize(obs_array.get_obstacles().size());
        int unassigned_obs_num = 0;
        for (size_t i = 0; i < obs_used.size(); i++){
            if (obs_used[i] == false){
                unassigned_obs[unassigned_obs_num++] = i;
            }
        }
        unassigned_obs.resize(unassigned_obs_num);
        //std::cerr << "3" << std::endl;
    }

    template <typename OBSType>
    void assign_track_obs_property_match(
        tsinghua::dias::fusion::OBSFObsFrame<OBSType> &obs_array,
        SensorType type_name,
        std::vector<std::pair<int, int> > &assignment,
        std::vector<int> &unassigned_track,
        std::vector<int> &unassigned_obs){
        std::vector<std::vector<double> > association_mat;

        association_mat.resize(unassigned_track.size());
        //std::cout << "association_mat dim: " << unassigned_track.size() << " " << unassigned_obs.size() << std::endl;
        for (int i = 0; i < unassigned_track.size(); i++)
        {
            association_mat[i].resize(unassigned_obs.size());
            for (int j = 0; j < unassigned_obs.size(); j++)
            {
                association_mat[i][j] = 0;
            }
        }
        /*
        tsinghua::dias::fusion::VectorUtil::init_2d_vector(
            association_mat,
            unassigned_track.size(),
            unassigned_obs.size(),
            0);
            */
        compute_assiciation_mat(obs_array, type_name,
            unassigned_track, unassigned_obs, association_mat); //problem detected! jxy 0517

        std::vector<int> unassigned_track_idx;
        std::vector<int> unassigned_obs_idx;
        /*
        if (association_mat.size() != 0 && association_mat[0].size() != 0){
            tsinghua::dias::fusion::CommonUtil::minimize_assignment(
                association_mat, unassigned_track_idx, unassigned_obs_idx);
        }
        */
        for(int i=0;i<unassigned_track.size();i++){unassigned_track_idx.push_back(i);}
        for(int i=0;i<unassigned_obs.size();i++){unassigned_obs_idx.push_back(i);}

        double max_dist = tsinghua::dias::fusion::FUSION_THRES;
        double max_dist_cam = tsinghua::dias::fusion::FUSION_PIXEL_THRES;
        double max_dist_cam_plus = tsinghua::dias::fusion::FUSION_PIXEL_THRES+5;

        int assignments_num = assignment.size();
        //std::cout << "assignment_num before property match: " << assignments_num << std::endl; //theoretically zero when id match is closed. jxy 0522
        assignment.resize(assignments_num + unassigned_track_idx.size());
        //std::cout << "_obs_track.size(): " << _obs_track.size() << std::endl;

        // old match algorithm
        // for (int i = 0; i < unassigned_track_idx.size(); ++i) {
        //     for (int j = 0; j < unassigned_obs_idx.size(); ++j){
        //         if (association_mat[unassigned_track_idx[i]][unassigned_obs_idx[j]] < max_dist) {
        //             assignment[assignments_num++] =
        //                 std::make_pair(unassigned_track[unassigned_track_idx[i]],
        //                                 unassigned_obs[unassigned_obs_idx[j]]);

        //             //unassigned_track[unassigned_track_idx[i]] = -1; //those who are assigned will be removed from the unassigned_track.
        //             //unassigned_obs[unassigned_obs_idx[j]] = -1;
        //             break; //for safety. jxy 0518
        //         }
        //     }

        // }

        /*Different tracks are associated with the same point,keep track of more iterations*/
        std::vector<float> track_smallest_position;
        std::vector<bool> track_smallest_flag;
        track_smallest_position.resize(unassigned_track_idx.size());
        track_smallest_flag.resize(unassigned_track_idx.size());

        for (int i = 0; i < unassigned_track_idx.size(); ++i)
        {
            track_smallest_flag[i] = 1;
            std::vector<double> temp_v = association_mat[i];
            std::vector<double>::iterator smallest = std::min_element(temp_v.begin(), temp_v.end());
            //std::cout << "Min element of track " << i << " is " << *smallest << " at position " << std::distance(temp_v.begin(), smallest) << std::endl;
            track_smallest_position[i] = std::distance(temp_v.begin(), smallest);
            
            //std::cout << association_mat[unassigned_track_idx[i]][unassigned_obs_idx[track_smallest_position[i]]] << std::endl;
        }

        // std::cout << "track assignment.x " << _obs_track[assignment[i].first].get_obs_radar()->_obs_position._x << " obs_assignment.x " << obs_array.get_obstacles()[assignment[i].second]._obs_position._x << std::endl;
        // std::cout << "track assignment.y " << _obs_track[assignment[i].first].get_obs_radar()->_obs_position._y << " obs_assignment.y " << obs_array.get_obstacles()[assignment[i].second]._obs_position._y << std::endl;

        //find repeat biggests
        if(obs_array.get_obstacles()[0]._sensor_type==7){
            for (int i = 0; i < unassigned_track_idx.size(); ++i){
                for (int j = i+1; j < unassigned_track_idx.size(); ++j){
                    if (track_smallest_position[i] == track_smallest_position[j]){
                        // if (_obs_track[unassigned_track[unassigned_track_idx[i]]].get_obs_radar()->flag_repeat > _obs_track[unassigned_track[unassigned_track_idx[j]]].get_obs_radar()->flag_repeat){
                        //     track_smallest_flag[j]=0;
                        // }
                 //       float a1 = atan2(_obs_track[unassigned_track[unassigned_track_idx[i]]].get_obs_radar()->_obs_position._y,_obs_track[unassigned_track[unassigned_track_idx[i]]].get_obs_radar()->_obs_position._x);
                   //     float b1 = atan2(_obs_track[unassigned_track[unassigned_track_idx[j]]].get_obs_radar()->_obs_position._y,_obs_track[unassigned_track[unassigned_track_idx[j]]].get_obs_radar()->_obs_position._x);
                        // std::cout << "obs theta:" << obs_array.get_obstacles()[unassigned_obs[unassigned_obs_idx[ track_smallest_position[j]]]]._velocity._z <<std::endl;
                        // std::cout << "track theta1:" << a1 <<std::endl;
                        // std::cout << "track theta2:" << b1 <<std::endl;

                        // if ( fabs(a1-obs_array.get_obstacles()[unassigned_obs[unassigned_obs_idx[ track_smallest_position[j]]]]._velocity._z) < fabs(b1-obs_array.get_obstacles()[unassigned_obs[unassigned_obs_idx[ track_smallest_position[j]]]]._velocity._z)){
                        //     track_smallest_flag[j]=0;
                        // std::vector<tsinghua::dias::fusion::OBSFObs3d,\
                        //     std::allocator<tsinghua::dias::fusion::OBSFObs3d> >
                        auto     _obs_array = obs_array.get_obstacles();
                        int index = unassigned_obs[unassigned_obs_idx[ track_smallest_position[j]]];
                        Obs3dPtr obs_track_i = _obs_track[unassigned_track[unassigned_track_idx[i]]].get_obs_radar();
                        Obs3dPtr obs_track_j = _obs_track[unassigned_track[unassigned_track_idx[j]]].get_obs_radar();
                        // float a2 = sqrt(pow(_obs_array._x - obs_track_i->_obs_position._x,2) + pow(_obs_array._y - obs_track_i->_obs_position._y,2));
                        // float b2 = sqrt(pow(_obs_array._x - obs_track_j->_obs_position._x,2) + pow(_obs_array._y - obs_track_j->_obs_position._y,2));
                    
                        // std::cout << "track theta1:" << a2 <<std::endl;
                        // std::cout << "track theta2:" << b2 <<std::endl;
                        double c= 0.66;
                        double omega_i =(obs_track_i->_obs_position._x-LRR_F_pos_x) *-0.00059703 +obs_track_i->_obs_position._y *-0.000050721 + c*0.00010254-0.0016;
         
                        double a_v_i= ((obs_track_i->_obs_position._x-LRR_F_pos_x)*-0.22384 + obs_track_i->_obs_position._y* 0.24574 +c*0.045906 -0.5648) / omega_i ;
                        double a_u_i= ((obs_track_i->_obs_position._x-LRR_F_pos_x) * -0.11899 +obs_track_i->_obs_position._y * -0.026076 +c*0.28690-0.8252) / omega_i ;  
                        double omega_j =(obs_track_j->_obs_position._x-LRR_F_pos_x) *-0.00059703 +obs_track_j->_obs_position._y *-0.000050721 + c*0.00010254-0.0016;
                        double a_v_j= ((obs_track_j->_obs_position._x-LRR_F_pos_x)*-0.22384 + obs_track_j->_obs_position._y* 0.24574 +c*0.045906 -0.5648) / omega_j ;
                        double a_u_j= ((obs_track_j->_obs_position._x-LRR_F_pos_x) * -0.11899 +obs_track_j->_obs_position._y * -0.026076 +c*0.28690-0.8252) / omega_j ;  

                        float a2 = sqrt(pow((384 - a_v_i)/768 * 100 - (384 - _obs_array[index]._v_center)/768 * 100,2)+ pow((240 - a_u_i)/480 * 60 - (240 - _obs_array[index]._u_center)/480 * 60,2)) ;
                        float b2 = sqrt(pow((384 - a_v_j)/768 * 100 - (384 - _obs_array[index]._v_center)/768 * 100,2)+ pow((240 - a_u_j)/480 * 60 - (240 - _obs_array[index]._u_center)/480 * 60,2)) ;
                        // std::cout << "track theta1:" << a2 <<std::endl;
                        // std::cout << "track theta2:" << b2 <<std::endl;
                        if ( a2 < b2){
                            track_smallest_flag[j]=0;
                        }else{
                            track_smallest_flag[i]=0;                        
                        }
                        if (a2>5){
                            track_smallest_flag[i]=0;
                        }
                        if (b2>5){
                            track_smallest_flag[j]=0;
                        }
                    }
                    // std::cout << "------------------------"  <<std::endl;

                }
            }
        }
        

        for (int i=0; i < unassigned_track_idx.size(); ++i){
            if (track_smallest_flag[i] == 1){
                if(obs_array.get_obstacles()[0]._sensor_type==7){
                    std::cout << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx camera mat dist:  " << association_mat[unassigned_track_idx[i]][unassigned_obs_idx[track_smallest_position[i]]] << std::endl;
                    // if(_obs_track[unassigned_track[unassigned_track_idx[i]]].get_obs_radar()->flag_repeat<10){  
                    //     if (association_mat[unassigned_track_idx[i]][unassigned_obs_idx[track_smallest_position[i]]] < max_dist_cam_plus){
                    //         assignment[assignments_num++] =
                    //             std::make_pair(unassigned_track[unassigned_track_idx[i]],unassigned_obs[unassigned_obs_idx[track_smallest_position[i]]]);                            
                    //     }
                    // }else{
                        if (association_mat[unassigned_track_idx[i]][unassigned_obs_idx[track_smallest_position[i]]] < max_dist_cam){
                            _obs_track[unassigned_track[unassigned_track_idx[i]]].get_obs_radar()->_ctype_update++;
                            assignment[assignments_num++] =
                                std::make_pair(unassigned_track[unassigned_track_idx[i]],unassigned_obs[unassigned_obs_idx[track_smallest_position[i]]]);
                        }
                    // }
                }else{
                    if (association_mat[unassigned_track_idx[i]][unassigned_obs_idx[track_smallest_position[i]]] < max_dist){
                        assignment[assignments_num++] =
                            std::make_pair(unassigned_track[unassigned_track_idx[i]],unassigned_obs[unassigned_obs_idx[track_smallest_position[i]]]);
                    }
                }   
            }           
        }


        // std::cout << "assignments_num: " << assignments_num << std::endl;

        assignment.resize(assignments_num);
        for (int i = 0; i < unassigned_track_idx.size(); ++i){
            for(int k=0;k<assignments_num;k++){
                if (unassigned_track[i]==assignment[k].first){
                    unassigned_track[i]=-1;
                }
            }
        }
        for (int j = 0; j < unassigned_obs_idx.size(); ++j){
            for (int k = 0; k < assignments_num; k++){
                if (unassigned_obs[j] == assignment[k].second){
                    unassigned_obs[j] = -1;
                }
            }
        }

        int unassigned_track_num = 0;
        for (int i = 0; i < unassigned_track.size(); ++i) {
            if (unassigned_track[i] >= 0) { //upper: = -1 removed those that are assigned in the property match step
                unassigned_track[unassigned_track_num++] = unassigned_track[i];
            }
        }
        unassigned_track.resize(unassigned_track_num);

        int unassigned_obs_num = 0;
        for (int i = 0; i < unassigned_obs.size(); ++i) {
            if (unassigned_obs[i] >= 0) {
                unassigned_obs[unassigned_obs_num++] = unassigned_obs[i];
            }
        }
        unassigned_obs.resize(unassigned_obs_num);
    }

    template <typename OBSType>
    void compute_assiciation_mat(
        tsinghua::dias::fusion::OBSFObsFrame<OBSType> &obs_array,
        SensorType type_name,
        std::vector<int> &unassigned_track,
        std::vector<int> &unassigned_obs,
        std::vector<std::vector<double> > &assiciation_mat
        ){
        tsinghua::dias::fusion::OBSFObsFrame<OBSType> unassigned_track_sync;
        
        sync_unassigned_track_array(unassigned_track,
            obs_array.get_header(),
            unassigned_track_sync);

        double time_stamp = obs_array.get_obstacles()[0]._header._stamp.to_second();
        float theta_new = obs_array.get_obstacles()[0]._course_angle_rad;

        for (size_t i = 0; i < unassigned_track.size(); i++){
            for (int j = 0; j < unassigned_obs.size(); j++){
                const OBSType &track_obs = unassigned_track_sync.get_obstacles()[i];
                const OBSType &obs_obs = obs_array.get_obstacles()[unassigned_obs[j]];
                //tsinghua::dias::fusion::CommonUtil::compute_polygon_center(track_obs, track_cnt);
                double track_cnt[4],obs_cnt[4];
                if(obs_obs._sensor_type != 7){               
                    double radar_time = track_obs._header._stamp.to_second();
                    double time_diff = time_stamp - radar_time;
                    float x2 = track_obs._obs_position._x + track_obs._velocity._x * time_diff;
                    float y2 = track_obs._obs_position._y + track_obs._velocity._y * time_diff;
                    float theta2 = theta_new - track_obs._course_angle_rad;

                    track_cnt[0] = x2*cosf(theta2)+y2*sinf(theta2);
                    track_cnt[1] = y2*cosf(theta2)-x2*sinf(theta2);
                    track_cnt[2] = track_obs._obs_position._z; //predict track before calculating association mat
                    track_cnt[3] = 0;
                    if(track_obs._sensor_type == 1 || track_obs._sensor_type == 2 ||
                        track_obs._sensor_type == 4 || track_obs._sensor_type == 5){
                        track_cnt[1] = track_obs._obs_position._y;
                        track_cnt[0] = track_obs._obs_position._x;                      
                    }
                    // track_cnt[1] = track_obs._obs_position._y;
                    // track_cnt[0] = track_obs._obs_position._x;

                    double c= 0.66;
                    if(obs_obs._sensor_type == 0 || obs_obs._sensor_type == 6){
                        double omega =(track_cnt[0]-LRR_F_pos_x) *-0.00059703 +track_cnt[1] *-0.000050721 + c*0.00010254-0.0016;
                        unassigned_track_sync.get_obstacles()[i]._v_center= ((track_cnt[0]-LRR_F_pos_x)*-0.22384 + track_cnt[1]* 0.24574 +c*0.045906 -0.5648) / omega ;
                        unassigned_track_sync.get_obstacles()[i]._u_center= ((track_cnt[0]-LRR_F_pos_x) * -0.11899 +track_cnt[1] * -0.026076 +c*0.28690-0.8252) / omega ;   
                    }
                    obs_cnt[0] = obs_obs._obs_position._x;
                    obs_cnt[1] = obs_obs._obs_position._y;
                    obs_cnt[2] = obs_obs._obs_position._z;
                    obs_cnt[3] = 0;
                }else{
                    double c= 0.66;
                    double omega =(track_obs._obs_position._x-LRR_F_pos_x) *-0.00059703 +track_obs._obs_position._y *-0.000050721 + c*0.00010254-0.0016;
                    // track_cnt[0]= ((track_obs._obs_position._x-LRR_F_pos_x)*-0.22384 + track_obs._obs_position._y* 0.24574 +c*0.045906 -0.5648) / omega ;
                    // track_cnt[1]= ((track_obs._obs_position._x-LRR_F_pos_x) * -0.11899 +track_obs._obs_position._y * -0.026076 +c*0.28690-0.8252) / omega ;  
                    double a_v= ((track_obs._obs_position._x-LRR_F_pos_x)*-0.22384 + track_obs._obs_position._y* 0.24574 +c*0.045906 -0.5648) / omega ;
                    double a_u= ((track_obs._obs_position._x-LRR_F_pos_x) * -0.11899 +track_obs._obs_position._y * -0.026076 +c*0.28690-0.8252) / omega ;  

                    track_cnt[0] = (384 - a_v)/768 * 100;
                    track_cnt[1] = (240 - a_u)/480 * 60;
                    track_cnt[2] = track_obs._obs_position._x;
                    track_cnt[3] = track_obs._obs_position._y;
                    
                    obs_cnt[0] =(384 - obs_obs._v_center)/768 * 100;
                    obs_cnt[1] = (240 - obs_obs._u_center)/480 * 60;
                    obs_cnt[2] = obs_obs._obs_position._x;
                    obs_cnt[3] = obs_obs._obs_position._y; 

                }

                double diff_pos[4];
                diff_pos[0] = track_cnt[0] - obs_cnt[0];
                diff_pos[1] = track_cnt[1] - obs_cnt[1];
                diff_pos[2] = track_cnt[2] - obs_cnt[2];
                diff_pos[3] = track_cnt[3] - obs_cnt[3];

                //diff_pos[2] = track_cnt[2] - obs_cnt[2]; //problem detected: all 3d should be replaced by 2d. jxy 0517
                double w1=1.0,w2= 1.0f;
                double dist_pos = 0.0f;
                if(obs_obs._sensor_type != 7){
                    dist_pos = sqrt(pow(diff_pos[0], 2) + pow(diff_pos[1], 2));
                }else{
                    // std::cout << "diff_pos_pixel " << i << ": " << diff_pos[0] << " " << diff_pos[1] << std::endl;
                    // std::cout << "diff_pos_world " << i << ": " << diff_pos[2] << " " << diff_pos[3] << std::endl;
                    // // dist_pos = w1*sqrt(pow(diff_pos[0], 2) + pow(diff_pos[1], 2))+w2*sqrt(pow(diff_pos[2], 2) + pow(diff_pos[3], 2));
                    dist_pos = w1*sqrt(pow(diff_pos[2], 2) + pow(diff_pos[3], 2));//+w2*sqrt(pow(diff_pos[2], 2) + pow(diff_pos[3], 2));
                    // std::cout << "diff_pos : " << dist_pos << std::endl;
                }
                
                assiciation_mat[i][j] = dist_pos;
            }
        }
    }
    template <typename OBSType>
    void sync_unassigned_track_array(
        std::vector<int> &unassigned_track,
        const tsinghua::dias::fusion::OBSFHeader &header,
        tsinghua::dias::fusion::OBSFObsFrame<OBSType> &obs_array_sync
        ){
        obs_array_sync.set_header(header);
        std::vector<OBSType> local_obs;
        local_obs.resize(unassigned_track.size());
        
        for (size_t i = 0; i < unassigned_track.size(); i++){
            const OBSType &obs = *(_obs_track[unassigned_track[i]].get_obs_radar());
            local_obs[i]=obs;
            //OBSType &r_obs = obs_array_sync.get_obstacles()[i];
            //tsinghua::dias::fusion::CommonUtil::compute_property(obs, header, r_obs);
            //tsinghua::dias::fusion::CommonUtil::deep_copy_obs(obs, r_obs); //problem detected: local_obs is not initialized, following deep_copy is not defined. jxy 0517
        }
        obs_array_sync.set_obstacles(local_obs);
    }

    void set_lane_detection_data(std::vector<ros_fusion::ld_Coeff> l_s, std::vector<int> ad_l, double l_t_s){
        _lane_sequence = l_s;
        _AutoDrive_lane = ad_l;
        _lane_time_stamp = l_t_s;
        // std::cout << "5" << std::endl;
    }

    int lane_interior_point_judge(OBFSVector3d obs_position){
        int lane_ww = 111;
        float x = obs_position._x;
        float y = obs_position._y;
        std::vector<bool> judge_number;// 0 0 1 1 
        int flag_judge_0 = 3,flag_judge_1 = 3;
        // std::cout << "========================" << std::endl;
        if(_lane_sequence.size()!=0 && _AutoDrive_lane.size()!=0){
            for(int i=0; i<_lane_sequence.size(); i++){
                float Y =  _lane_sequence[i].a*pow(x,3)+_lane_sequence[i].b*pow(x,2)
                            +_lane_sequence[i].c*x+_lane_sequence[i].d;
                if(Y>=y){
                    judge_number.push_back(0);
                    flag_judge_0 = 0;
                    // std::cout << "5: " << flag_judge_0 << std::endl;
                }else{
                    judge_number.push_back(1);
                    flag_judge_1 = 1;
                    // std::cout << "6: " << flag_judge_1 << std::endl;
                }
            }
            // std::cout << "7" << std::endl;
            if(flag_judge_0==0 && flag_judge_1==3){ lane_ww = -9;
            }else if(flag_judge_0==3 && flag_judge_1==1){ lane_ww = 9;
            }else{
                for(int j=0; j<judge_number.size(); j++){
                    if(judge_number[j] == 1){
                        lane_ww = _AutoDrive_lane[j-1];
                        // std::cout << "7:  " << _AutoDrive_lane[j-1] << std::endl;
                        break;
                    }       
                }
            }
        }else{
            lane_ww = 11;
        }    
        // std::cout << "8:  " << lane_ww << std::endl;
        return lane_ww;
    }

    const std::vector<tsinghua::dias::fusion::OBSFUncertaintyTrack> &get_fusion_result()const{
        return _obs_track;
    }

    tsinghua::dias::fusion::OBSFObs3dFrame& get_lidar_frame() {
        return _lidar_obs_array;
    }

    tsinghua::dias::fusion::OBSFObs3dFrame& get_radar_frame() {
        return _radar_obs_array;
    }

    int& get_obs_array_sensor_type() {
        return obs_array_sensor_type;
    } 

    std::vector<tsinghua::dias::fusion::OBSFUncertaintyTrack>& get_tracks() {
        return _obs_track;
    }

    void init_id_arr(){
        for (int i = 1; i < 512;i++){
            id_arr.push_back(i);
        }
    }
    int get_id(){
        if (id_arr.size() > 0){
        int id = id_arr[0];
        id_arr.erase(id_arr.begin());
        return id;
        }
        else 
        std::cout << "array not long enough" << std::endl;
        return 0;
    }

    void check_id(int id){
        id_arr.push_back(id);
    }
private:
    tsinghua::dias::fusion::OBSFObs3dFrame _camera_obs_array;
    std::mutex _came_obs_array_mutex;
    tsinghua::dias::fusion::OBSFObs3dFrame _lidar_obs_array;
    std::mutex _lidar_obs_array_mutex;
    tsinghua::dias::fusion::OBSFObs3dFrame _radar_obs_array;
    std::mutex _radar_obs_array_mutex;
    std::vector<tsinghua::dias::fusion::OBSFUncertaintyTrack> _obs_track;
    std::mutex _fusion_mutex;
    std::vector<int> id_arr; 
    bool sensor_type_f_or_b;
    int obs_array_sensor_type;
    std::vector<ros_fusion::ld_Coeff> _lane_sequence;
    std::vector<int> _AutoDrive_lane;
    double _lane_time_stamp = -3;
};

}//
}//
}//

#endif
