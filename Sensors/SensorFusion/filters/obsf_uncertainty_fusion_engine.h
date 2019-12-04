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
#include "OpenICV/structure/structureIMU.h"
#include "OpenICV/structure/structureFusionData.h"
#include "OpenICV/structure/structureLaneData.h"
#include "OpenICV/Core/icvTime.h"
#include "../objects/obsf_fusion_track.h"
#include "../objects/obsf_obs_frame.h"
#include "../objects/obsf_define.h"
#include <opencv2/opencv.hpp>

using namespace icv ;
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



float LRR_F_pos_x= 3.7;
float LRR_F_pos_y= 0.0;
float LRR_B_pos_x= -1.03;
float LRR_B_pos_y= 0.2;



int f_control_mode = 255; //0 unlimited, 1 limited area to create and delete
int b_control_mode = 255; //0 unlimited, 1 limited area to create and delete
class OBSFUncertaintyFusionEngine{
public:
    typedef std::shared_ptr<OBSFObs3d> Obs3dPtr;

    OBSFUncertaintyFusionEngine(){
        init_id_arr();
    }
    ~OBSFUncertaintyFusionEngine(){}

    void set_control_mode(int i, int j){
        sensor_type_f_or_b = j;
        if(j == 0){
            f_control_mode = i;
            
        }else{
            b_control_mode = i;
        }
    }
    
    void process_camera_frame(
        const tsinghua::dias::fusion::OBSFObs3dFrame &obs_array){
        _came_obs_array_mutex.lock();
        _camera_obs_array = obs_array;
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

    template <typename OBSType>
    void fusion(
        tsinghua::dias::fusion::OBSFObsFrame<OBSType> &obs_array,
        const tsinghua::dias::fusion::SensorType &sensor_type){
        _fusion_mutex.lock();
        std::vector<std::pair<int, int> > assignment;
        std::vector<int> unassigned_track;
        std::vector<int> unassigned_obs;

        assign_track_obs_id_match(obs_array, sensor_type, assignment,
            unassigned_track, unassigned_obs);

        assign_track_obs_property_match(obs_array, sensor_type, assignment,
            unassigned_track, unassigned_obs);
        
        update_assigned_track(obs_array, sensor_type, assignment);

        update_unassigned_track(obs_array, sensor_type, unassigned_track);

        delete_lost_track();
        
        create_new_track(obs_array, sensor_type, unassigned_obs);

        _fusion_mutex.unlock();
    }

    double valueAt(std::vector<float> &f, float x)
    {
        float ans = 0.f;
        for (int i = (int)f.size() - 1; i >= 0; --i)
            ans = ans * x + f[i];
        return ans;
    }

    void predict_all_track_states(const tsinghua::dias::fusion::OBSFHeader &header){//double observation_time
        for (size_t i = 0; i < _obs_track.size(); i++){
            _obs_track[i].prediction(header);
        }
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
                        obs_array.get_obstacles()[unassigned_obs[i]]._obs_id=get_id();
                        _obs_track.push_back(tsinghua::dias::fusion::OBSFUncertaintyTrack(
                            obs_array.get_obstacles()[unassigned_obs[i]], sensor_type));          
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
        
        if(1){ // jxy 20190612 freed camera
            for (int i = 0; i < unassigned_obs.size(); i++){                     
                    obs_array.get_obstacles()[unassigned_obs[i]].P.setZero(); //Really confusing! check in kalman filter and found the problem. Camera P is not set! jxy 20190612
                    obs_array.get_obstacles()[unassigned_obs[i]].P(0,0) = obs_array.get_obstacles()[unassigned_obs[i]].rangex_rms;
                    obs_array.get_obstacles()[unassigned_obs[i]].P(1,1) = obs_array.get_obstacles()[unassigned_obs[i]].rangey_rms;
                    obs_array.get_obstacles()[unassigned_obs[i]].P(2,2) = obs_array.get_obstacles()[unassigned_obs[i]].speedy_rms;
                    obs_array.get_obstacles()[unassigned_obs[i]].P(3,3) = obs_array.get_obstacles()[unassigned_obs[i]].speedy_rms;

                    tsinghua::dias::fusion::hist_track_obs temp_hist_obs;
                    tsinghua::dias::fusion::hist_track_obs temp_hist_track;

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

                    temp_hist_track._sensor_type = obs_array.get_obstacles()[unassigned_obs[i]]._sensor_type;
                    obs_array.get_obstacles()[unassigned_obs[i]]._hist_states.push_front(temp_hist_track); // store the history in hist_states;
                    
                    temp_hist_obs = temp_hist_track;
                    obs_array.get_obstacles()[unassigned_obs[i]]._hist_obs.push_front(temp_hist_obs); // store the history in hist_obs;
            }

            for (int i = 0; i < unassigned_obs.size(); i++){
                obs_array.get_obstacles()[unassigned_obs[i]]._obs_id=get_id();
                double lane_timp_flag = SyncClock::time_s() - _lane_time_stamp;
                if(lane_timp_flag > 1){
                    _lane_sequence.clear();
                    _AutoDrive_lane.clear();
                }
               
            }
        }
        int sensor_type_num = obs_array.get_obstacles()[0]._sensor_type;
        float lrr_mid_x, lrr_mid_y, ldr_right_slope, ldr_left_slope;
        int control_mode;
        if(sensor_type_f_or_b == 0){
            lrr_mid_x = LRR_F_pos_x;
            lrr_mid_y = LRR_F_pos_y;
            ldr_right_slope = lidar_right_slope;
            ldr_left_slope = lidar_left_slope;
            control_mode = f_control_mode;
        }else{
            lrr_mid_x = LRR_B_pos_x;
            lrr_mid_y = LRR_B_pos_y;
            ldr_right_slope = B_lidar_left_slope;
            ldr_left_slope = B_lidar_right_slope;
            control_mode = b_control_mode;
        }
        create_track_area_partition_function(control_mode, sensor_type_num, obs_array,sensor_type,
                                unassigned_obs, lrr_mid_x, lrr_mid_y, ldr_right_slope, ldr_left_slope);

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
            if (flag_track_distance[i] == 0){
                _obs_track[track_num++] = _obs_track[i]; //assign pair will get useless after this. jxy 0522
            }else{
                check_id(_obs_track[i].get_obs_radar()->_obs_id);
            }
        }
        _obs_track.resize(track_num);
        obs_array_sensor_type = obs_array.get_obstacles()[0]._sensor_type;
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
        _obs_track.resize(track_num);
    }

    template <typename OBSType>
    void store_history_tracks_obs(int i, OBSType &ob
        ){        
            tsinghua::dias::fusion::hist_track_obs temp_hist_track;
            temp_hist_track._header = _obs_track[i].get_obs_radar()->_header._stamp.to_second();
            temp_hist_track._obs_position._x = _obs_track[i].get_obs_radar()->_obs_position._x;
            temp_hist_track._obs_position._y = _obs_track[i].get_obs_radar()->_obs_position._y;
            temp_hist_track._velocity._x = _obs_track[i].get_obs_radar()->_velocity._x;
            temp_hist_track._velocity._y = _obs_track[i].get_obs_radar()->_velocity._y;
            temp_hist_track.P = _obs_track[i].get_obs_radar()->P;
            temp_hist_track._sensor_type = _obs_track[i].get_obs_radar()->_sensor_type;
            _obs_track[i].get_obs_radar()->_hist_states.push_front(temp_hist_track);   // store the history in hist_states;
            if(_obs_track[i].get_obs_radar()->_hist_states.size() > 10){  _obs_track[i].get_obs_radar()->_hist_states.pop_back();  }
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
            _obs_track[i].get_obs_radar()->_hist_obs.push_front(temp_hist_obs); // store the history in hist_obs;
            if(_obs_track[i].get_obs_radar()->_hist_obs.size() > 10){ _obs_track[i].get_obs_radar()->_hist_obs.pop_back(); }
        }

    template <typename OBSType>
    void update_assigned_track(
        tsinghua::dias::fusion::OBSFObsFrame<OBSType> &obs_array,
        const tsinghua::dias::fusion::SensorType &sensor_type,
        const std::vector<std::pair<int, int> > &assignment
        ){
        double time_stamp = obs_array.get_header()._stamp.to_second();

        for (size_t i = 0; i < assignment.size(); i++){
            auto _obs_array = obs_array.get_obstacles()[assignment[i].second];
            auto obs_track = _obs_track[assignment[i].first].get_obs_radar();

            obs_track->flag_repeat++;
            _obs_array.flag_repeat = obs_track->flag_repeat;
            _obs_array._obs_id = obs_track->_obs_id;
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
                _obs_array._obs_theta = obs_track->_obs_theta;
            }

            if(_obs_array._sensor_type == 7){
                _obs_array._velocity = obs_track->_velocity;
            }
            auto camera_obj_type = _obs_array._object_type; 
            
            if(_obs_array._sensor_type != 7){
                //the camera should have the right to update position! or else if there were only camera, how would fusion work? jxy 20190612
                _obs_array._object_type = obs_track->_object_type;
            }
            if(_obs_array._sensor_type != 7 || f_control_mode == 111){ // keep the way of old versions. jxy 20190612
                _obs_track[assignment[i].first].set_obs(*obs_track, _obs_array, sensor_type); //kalman filter
            }

            // std::cout << "flag: " << obs_track->flag_repeat << std::endl;

            
            if(_obs_array._sensor_type == 7){ //camera X kalman
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
            
            store_history_tracks_obs(assignment[i].first, _obs_array);
        }
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
                    int sensor_type_num = obs_array.get_obstacles()[0]._sensor_type;
                    int ii = i;
                    float lrr_mid_x, lrr_mid_y, ldr_right_slope, ldr_left_slope, mid_max_x;
                    int control_mode;
                    if(sensor_type_f_or_b == 0){
                        lrr_mid_x = LRR_F_pos_x;
                        lrr_mid_y = LRR_F_pos_y;
                        mid_max_x = lidar_x_max_dist;
                        ldr_right_slope = lidar_right_slope;
                        ldr_left_slope = lidar_left_slope;
                        control_mode = f_control_mode;
                    }else{
                        lrr_mid_x = LRR_B_pos_x;
                        lrr_mid_y = LRR_B_pos_y;
                        mid_max_x = B_lidar_x_max_dist;
                        ldr_right_slope = B_lidar_left_slope;  
                        ldr_left_slope = B_lidar_right_slope;
                        control_mode = b_control_mode;
                    }
                    float track_x = _obs_track[unassigned_track[i]].get_obs_radar()->_obs_position._x;
                    float track_y = _obs_track[unassigned_track[i]].get_obs_radar()->_obs_position._y;
                    float track_slope = (_obs_track[unassigned_track[i]].get_obs_radar()->_obs_position._x - lrr_mid_x) / (_obs_track[unassigned_track[i]].get_obs_radar()->_obs_position._y);
                    delete_track_area_partition_function(control_mode, sensor_type_num, sensor_type,
                                unassigned_track, mid_max_x,track_x, track_y, ldr_right_slope, ldr_left_slope, track_slope, ii);                              
                }
            } else {
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
                continue;
            }
        }

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
            if (obs_used[i] == false){
                unassigned_obs[unassigned_obs_num++] = i;
            }
        }
        unassigned_obs.resize(unassigned_obs_num);
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
                continue;
            }

            for (size_t j = 0; j < obs_array.get_obstacles().size(); j++){
                if (obs->_obs_id == obs_array.get_obstacles()[j]._obs_id){
                    assignment[assignment_num++] = std::make_pair(i, j);
                    track_used[i] = true;
                    obs_used[j] = true;
                }
            }
        }
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
            if (obs_used[i] == false){
                unassigned_obs[unassigned_obs_num++] = i;
            }
        }
        unassigned_obs.resize(unassigned_obs_num);
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
        for (int i = 0; i < unassigned_track.size(); i++)
        {
            association_mat[i].resize(unassigned_obs.size());
            for (int j = 0; j < unassigned_obs.size(); j++)
            {
                association_mat[i][j] = 0;
            }
        }

        compute_association_mat(obs_array, type_name, unassigned_track, unassigned_obs, association_mat);

        std::vector<int> unassigned_track_idx;
        std::vector<int> unassigned_obs_idx;

        for(int i=0;i<unassigned_track.size();i++){unassigned_track_idx.push_back(i);}
        for(int i=0;i<unassigned_obs.size();i++){unassigned_obs_idx.push_back(i);}

        double max_dist = tsinghua::dias::fusion::FUSION_THRES;
        double max_dist_cam = tsinghua::dias::fusion::FUSION_PIXEL_THRES;
        double max_dist_cam_plus = tsinghua::dias::fusion::FUSION_PIXEL_THRES+5;

        int assignments_num = assignment.size();
        assignment.resize(assignments_num + unassigned_track_idx.size());

        //Different object type calls for different matching methods.
        if(obs_array.get_obstacles()[0]._sensor_type != 7){ //Match by xy.
            for (int i = 0; i < unassigned_track_idx.size(); ++i)
            {
                /*TODO: Different tracks are associated with the same object, keep track of more iterations*/
                std::vector<float> track_smallest_position;
                std::vector<bool> track_smallest_flag;
                track_smallest_position.resize(unassigned_track_idx.size());
                track_smallest_flag.resize(unassigned_track_idx.size());

                for (int i = 0; i < unassigned_track_idx.size(); ++i)
                {
                    track_smallest_flag[i] = 1;
                    std::vector<double> temp_v = association_mat[i];
                    std::vector<double>::iterator smallest = std::min_element(temp_v.begin(), temp_v.end());
                    track_smallest_position[i] = std::distance(temp_v.begin(), smallest);
                }
                if (association_mat[unassigned_track_idx[i]][unassigned_obs_idx[track_smallest_position[i]]] < max_dist)
                {
                    assignment[assignments_num++] =
                        std::make_pair(unassigned_track[unassigned_track_idx[i]], unassigned_obs[unassigned_obs_idx[track_smallest_position[i]]]);
                }
            }
        }

        std::vector<bool> valid_track_flag;
        valid_track_flag.resize(unassigned_track_idx.size());
        double v_center;
        double u_center;
        double width_pixel;
        double height_pixel;

        if(obs_array.get_obstacles()[0]._sensor_type == 7){ //Match by object's uvxy.
            img_lane = cv::Mat(cv::Size(512, 256), CV_8UC3, cv::Scalar(255, 255, 255));
            cv::Point2f dot_p;
            std::vector<float> coeff;
            coeff.resize(0);

            for (int i = 0; i < _lane_sequence.size(); i++)
            {
                coeff.resize(0);
                // coeff.push_back(_lane_sequence[i].d_img);
                coeff.push_back(_lane_sequence[i].c_img);
                coeff.push_back(_lane_sequence[i].b_img);
                coeff.push_back(_lane_sequence[i].a_img);

                for (int yy = 100; yy < 256; ++yy)
                {
                    dot_p.y = yy;
                    dot_p.x = valueAt(coeff, dot_p.y);

                    cv::circle(img_lane, dot_p, 2, cv::Scalar(0, 255, 0), 1, 8, 0); //valueAt is the fitted result
                }
            }

            double v_center;
            double u_center;
            double width_pixel;
            double height_pixel;

            for (int i = 0; i < obs_array.get_obstacles().size(); i++)
            {
                v_center = obs_array.get_obstacles()[i]._v_center;
                u_center = obs_array.get_obstacles()[i]._u_center;
                width_pixel = obs_array.get_obstacles()[i]._width_pixel;
                height_pixel = obs_array.get_obstacles()[i]._height_pixel;
                // std::cout << "vuwh: " << v_center << " " << u_center << " " << width_pixel << " " << height_pixel << std::endl;
                cv::rectangle(img_lane, cv::Rect(v_center -128 - width_pixel / 2, u_center - 150 - height_pixel / 2, width_pixel, height_pixel), cv::Scalar(0, 0, 0), 1, 8, 0);
            }

            
            for (int j = 0; j < unassigned_obs_idx.size(); ++j){
                int valid_count = 0;
                v_center = obs_array.get_obstacles()[j]._v_center;
                u_center = obs_array.get_obstacles()[j]._u_center;
                width_pixel = obs_array.get_obstacles()[j]._width_pixel;
                height_pixel = obs_array.get_obstacles()[j]._height_pixel;

                //several filters
                //1. track position to the 2Dbox and check if it is inside.
                for (int i = 0; i < unassigned_track_idx.size(); ++i)
                {
                    Obs3dPtr obs_track_i = _obs_track[unassigned_track[unassigned_track_idx[i]]].get_obs_radar();

                    double c = 0.66;
                    double omega_i = (obs_track_i->_obs_position._x - LRR_F_pos_x) * -0.00059703 + obs_track_i->_obs_position._y * -0.000050721 + c * 0.00010254 - 0.0016;

                    double a_v = ((obs_track_i->_obs_position._x - LRR_F_pos_x) * -0.22384 + obs_track_i->_obs_position._y * 0.24574 + c * 0.045906 - 0.5648) / omega_i;
                    double a_u = ((obs_track_i->_obs_position._x - LRR_F_pos_x) * -0.11899 + obs_track_i->_obs_position._y * -0.026076 + c * 0.28690 - 0.8252) / omega_i;
                    if(a_v>v_center-width_pixel/2 && a_v<v_center+width_pixel/2 && a_u>u_center-height_pixel/2 && a_u<u_center+height_pixel/2){
                        valid_track_flag[i] = 1;
                        valid_count=valid_count+1;
                    }
                    else
                    {
                        valid_track_flag[i] = 0;
                    }
                }
                if (valid_count == 0){
                    // std::cout << "Match failure!" << std::endl;
                    // std::cout << "vuwh: " << v_center << " " << u_center << " " << width_pixel << " " << height_pixel << std::endl;
                    break;
                }
                else if (valid_count == 1){
                    int haha;
                    for(int i=0;i< unassigned_track_idx.size(); ++i){
                        if(valid_track_flag[i]==1){
                            haha=i;
                            Obs3dPtr obs_track_i = _obs_track[unassigned_track[unassigned_track_idx[haha]]].get_obs_radar();

                            double c = 0.66;
                            double omega_i = (obs_track_i->_obs_position._x - LRR_F_pos_x) * -0.00059703 + obs_track_i->_obs_position._y * -0.000050721 + c * 0.00010254 - 0.0016;
                            double a_v = ((obs_track_i->_obs_position._x - LRR_F_pos_x) * -0.22384 + obs_track_i->_obs_position._y * 0.24574 + c * 0.045906 - 0.5648) / omega_i;
                            double a_u = ((obs_track_i->_obs_position._x - LRR_F_pos_x) * -0.11899 + obs_track_i->_obs_position._y * -0.026076 + c * 0.28690 - 0.8252) / omega_i;
                            cv::Point2f trackpoint;
                            trackpoint.x=a_v-128;
                            trackpoint.y=a_u-150;

                            assignment[assignments_num++] =
                                std::make_pair(unassigned_track[unassigned_track_idx[haha]], unassigned_obs[unassigned_obs_idx[j]]);
                            // std::cout << "Camera match succeeded:" << std::endl;
                            // std::cout << "vuwh: " << v_center << " " << u_center << " " << width_pixel << " " << height_pixel << std::endl;
                            // std::cout << "track xy:" << obs_track_i->_obs_position._x << " " << obs_track_i->_obs_position._y << std::endl
                            //           << std::endl;
                            cv::circle(img_lane, trackpoint, 2, cv::Scalar(255, 0, 0), 3, 8, 0);
                            break;
                        }
                    }
                    
                    break;
                }
                else{
                    // std::cout << "two or more matched, enter the next filter." << std::endl;
                    // std::cout << "vuwh: " << v_center << " " << u_center << " " << width_pixel << " " << height_pixel << std::endl;
                }

                //2. Check xy, the difference should not be too big.
                for (int i = 0; i < unassigned_track_idx.size(); ++i)
                {
                    if(valid_track_flag[i]==1){
                        Obs3dPtr obs_track_i = _obs_track[unassigned_track[unassigned_track_idx[i]]].get_obs_radar();
                        double dx = obs_track_i->_obs_position._x - obs_array.get_obstacles()[j]._obs_position._x;
                        double dy = obs_track_i->_obs_position._y - obs_array.get_obstacles()[j]._obs_position._y;
                        double distance = sqrt(pow(obs_track_i->_obs_position._y, 2) + pow(obs_track_i->_obs_position._x, 2));
                        if (sqrt(dx*dx+dy*dy)<=distance*0.1) //believe that the camera error is less than 10%
                        {
                            valid_track_flag[i] = 1;
                        }
                        else
                        {
                            valid_track_flag[i] = 0;
                            valid_count = valid_count - 1;
                        }
                    }
                }
                if (valid_count == 0)
                {
                    // std::cout << "Match failure!" << std::endl;
                    // std::cout << "vuwh: " << v_center << " " << u_center << " " << width_pixel << " " << height_pixel << std::endl;
                    break;
                }
                else if (valid_count == 1)
                {
                    int haha;
                    for (int i = 0; i < unassigned_track_idx.size(); ++i)
                    {
                        if (valid_track_flag[i] == 1)
                        {
                            haha = i;
                            Obs3dPtr obs_track_i = _obs_track[unassigned_track[unassigned_track_idx[haha]]].get_obs_radar();

                            double c = 0.66;
                            double omega_i = (obs_track_i->_obs_position._x - LRR_F_pos_x) * -0.00059703 + obs_track_i->_obs_position._y * -0.000050721 + c * 0.00010254 - 0.0016;
                            double a_v = ((obs_track_i->_obs_position._x - LRR_F_pos_x) * -0.22384 + obs_track_i->_obs_position._y * 0.24574 + c * 0.045906 - 0.5648) / omega_i;
                            double a_u = ((obs_track_i->_obs_position._x - LRR_F_pos_x) * -0.11899 + obs_track_i->_obs_position._y * -0.026076 + c * 0.28690 - 0.8252) / omega_i;
                            cv::Point2f trackpoint;
                            trackpoint.x = a_v-128;
                            trackpoint.y = a_u-150;

                            assignment[assignments_num++] =
                                std::make_pair(unassigned_track[unassigned_track_idx[haha]], unassigned_obs[unassigned_obs_idx[j]]);
                            // std::cout << "Camera match succeeded:" << std::endl;
                            // std::cout << "vuwh: " << v_center << " " << u_center << " " << width_pixel << " " << height_pixel << std::endl;
                            // std::cout << "track xy:" << obs_track_i->_obs_position._x << " " << obs_track_i->_obs_position._y << std::endl
                            //           << std::endl;
                            cv::circle(img_lane, trackpoint, 2, cv::Scalar(255, 0, 0), 3, 8, 0);
                            break;
                        }
                    }

                    break;
                }
                else
                {
                    // std::cout << "vuwh: " << v_center << " " << u_center << " " << width_pixel << " " << height_pixel << std::endl;
                }
            }
            // cv::imshow("lanes", img_lane);
            // cv::waitKey(1);
        }

        //Property match is completed.
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
    void compute_association_mat(
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
                    double c= 0;
                    double omega =(track_obs._obs_position._x-LRR_F_pos_x) *-0.00059703 +track_obs._obs_position._y *-0.000050721 + c*0.00010254-0.0016;
                    double a_v= ((track_obs._obs_position._x-LRR_F_pos_x)*-0.22384 + track_obs._obs_position._y* 0.24574 +c*0.045906 -0.5648) / omega ;
                    double a_u= ((track_obs._obs_position._x-LRR_F_pos_x) * -0.11899 +track_obs._obs_position._y * -0.026076 +c*0.28690-0.8252) / omega ;  

                    track_cnt[0] = (384 - a_v)/768 * 100;
                    track_cnt[1] = (240 - a_u)/480 * 60;
                    track_cnt[2] = track_obs._obs_position._x;
                    track_cnt[3] = track_obs._obs_position._y;

                    //jxy 20190605 TODO: it is not ensured that the obs has u and v. For example, a camera track with a radar obs.
                    obs_cnt[0] =(384 - obs_obs._v_center)/768 * 100;
                    obs_cnt[1] = (240 - obs_obs._u_center) / 480 * 60 + obs_obs._height_pixel / 2; //jxy 20190605 added half height: xy is transformed by the bottom of the bbox in uv.
                    obs_cnt[2] = obs_obs._obs_position._x;
                    obs_cnt[3] = obs_obs._obs_position._y; 

                }

                double diff_pos[4];
                diff_pos[0] = track_cnt[0] - obs_cnt[0];
                diff_pos[1] = track_cnt[1] - obs_cnt[1];
                diff_pos[2] = track_cnt[2] - obs_cnt[2];
                diff_pos[3] = track_cnt[3] - obs_cnt[3];
                double w1=1.0,w2= 1.0f;
                double dist_pos = 0.0f;
                if(obs_obs._sensor_type != 7){
                    dist_pos = sqrt(pow(diff_pos[0], 2) + pow(diff_pos[1], 2));
                }else{
                    dist_pos = w1*sqrt(pow(diff_pos[2], 2) + pow(diff_pos[3], 2));
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
        }
        obs_array_sync.set_obstacles(local_obs);
    }

    void set_lane_detection_data(std::vector<ld_Coeff> l_s, std::vector<int> ad_l, double l_t_s){
        _lane_sequence = l_s;
        _AutoDrive_lane = ad_l;
        _lane_time_stamp = l_t_s;
    }

    int lane_interior_point_judge(OBFSVector3d obs_position){
        int lane_ww = 111;
        float x = obs_position._x;
        float y = obs_position._y;
        std::vector<bool> judge_number;
        int flag_judge_0 = 3,flag_judge_1 = 3;
        if(_lane_sequence.size()!=0 && _AutoDrive_lane.size()!=0){
            for(int i=0; i<_lane_sequence.size(); i++){
                float Y =  _lane_sequence[i].a*pow(x,3)+_lane_sequence[i].b*pow(x,2)
                            +_lane_sequence[i].c*x+_lane_sequence[i].d;
                if(Y>=y){
                    judge_number.push_back(0);
                    flag_judge_0 = 0;
                }else{
                    judge_number.push_back(1);
                    flag_judge_1 = 1;
                }
            }
            if(flag_judge_0==0 && flag_judge_1==3){ lane_ww = -9;
            }else if(flag_judge_0==3 && flag_judge_1==1){ lane_ww = 9;
            }else{
                for(int j=0; j<judge_number.size(); j++){
                    if(judge_number[j] == 1){
                        lane_ww = _AutoDrive_lane[j-1];
                        break;
                    }       
                }
            }
        }else{
            lane_ww = 11;
        }    
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
    std::vector<ld_Coeff> _lane_sequence;
    std::vector<int> _AutoDrive_lane;
    double _lane_time_stamp = -3;
public:
    cv::Mat img_lane ;
};

}//
}//
}//

#endif
