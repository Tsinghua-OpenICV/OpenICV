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
#include "../objects/obsf_fusion_track.h"
#include "../objects/obsf_obs_frame.h"

#include "../objects/obsf_define.h"



namespace tsinghua {
namespace dias {
namespace fusion{

#define lidar_right_slope -0.6
#define lidar_left_slope 0.9
#define lidar_x_max_dist 65
#define LRR_F_pos_x 3.7

int control_mode = 255; //0 unlimited, 1 limited area to create and delete

class OBSFUncertaintyFusionEngine{
public:
    typedef std::shared_ptr<OBSFObs3d> Obs3dPtr;

    OBSFUncertaintyFusionEngine(){
        init_id_arr();
    }
    ~OBSFUncertaintyFusionEngine(){}
    void set_control_mode(int i){
        control_mode = i;
        // std::cout << "control_mode: " << control_mode << std::endl;
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

    //just delete radar obstacles
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
    void create_new_track(
        tsinghua::dias::fusion::OBSFObsFrame<OBSType> &obs_array,
        const tsinghua::dias::fusion::SensorType &sensor_type,
        std::vector<int>& unassigned_obs
        ){
        for (int i = 0; i < unassigned_obs.size(); i++){                     
                obs_array.get_obstacles()[unassigned_obs[i]].P.setZero();
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
        // if (sensor_type == tsinghua::dias::fusion::RADAR) {return;}
        /*radar detection zone limit
        Radar generated track limit, all the laser radar is reserved, the forward radar is retained 65 meters, 
        the right front radar is retained more than -0.6 slope, the left front radar is more than -0.6 slope.*/
        if (control_mode == -1 || obs_array.get_obstacles()[0]._sensor_type!=7){
            for (int i = 0; i < unassigned_obs.size(); i++){
                obs_array.get_obstacles()[unassigned_obs[i]]._obs_id=get_id();
                _obs_track.push_back(tsinghua::dias::fusion::OBSFUncertaintyTrack(
                    obs_array.get_obstacles()[unassigned_obs[i]], sensor_type));
            }
        }else if(control_mode == 111){  //lidar radar all died
            for (int i = 0; i < unassigned_obs.size(); i++){
                obs_array.get_obstacles()[unassigned_obs[i]]._obs_id=get_id();
                _obs_track.push_back(tsinghua::dias::fusion::OBSFUncertaintyTrack(
                    obs_array.get_obstacles()[unassigned_obs[i]], sensor_type));
            }
        }else{
            if(obs_array.get_obstacles()[0]._sensor_type==6){
                for (int i = 0; i < unassigned_obs.size(); i++){ 
                    obs_array.get_obstacles()[unassigned_obs[i]]._obs_id=get_id();        
                    _obs_track.push_back(tsinghua::dias::fusion::OBSFUncertaintyTrack(
                        obs_array.get_obstacles()[unassigned_obs[i]], sensor_type));          
                }
            }else if(obs_array.get_obstacles()[0]._sensor_type==1){
                for (int i = 0; i < unassigned_obs.size(); i++){
                    if (((obs_array.get_obstacles()[unassigned_obs[i]]._obs_position._x - LRR_F_pos_x) / obs_array.get_obstacles()[unassigned_obs[i]]._obs_position._y > lidar_right_slope) && (obs_array.get_obstacles()[unassigned_obs[i]]._obs_position._y < 0))
                    {
                        obs_array.get_obstacles()[unassigned_obs[i]]._obs_id=get_id();
                        _obs_track.push_back(tsinghua::dias::fusion::OBSFUncertaintyTrack(
                            obs_array.get_obstacles()[unassigned_obs[i]], sensor_type));          
                    }
                }
            }else if(obs_array.get_obstacles()[0]._sensor_type==2){
                for (int i = 0; i < unassigned_obs.size(); i++){
                    if (((obs_array.get_obstacles()[unassigned_obs[i]]._obs_position._x - LRR_F_pos_x) / obs_array.get_obstacles()[unassigned_obs[i]]._obs_position._y < lidar_left_slope) && (obs_array.get_obstacles()[unassigned_obs[i]]._obs_position._y > 0))
                    {
                        obs_array.get_obstacles()[unassigned_obs[i]]._obs_id=get_id();
                        _obs_track.push_back(tsinghua::dias::fusion::OBSFUncertaintyTrack(
                            obs_array.get_obstacles()[unassigned_obs[i]], sensor_type));          
                    }
                }
            }else if(obs_array.get_obstacles()[0]._sensor_type==0){
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
                
                if (track_distance < max_dist && vel_diff < 1){
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
        //std::cout << "_obs_track.size()" << _obs_track.size() << std::endl;
        _obs_track.resize(track_num);

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
    void update_assigned_track(
        tsinghua::dias::fusion::OBSFObsFrame<OBSType> &obs_array,
        const tsinghua::dias::fusion::SensorType &sensor_type,
        const std::vector<std::pair<int, int> > &assignment
        ){
        //std::cout << "assignment.size()" << assignment.size() << std::endl;
        double time_stamp = obs_array.get_header()._stamp.to_second();

        for (size_t i = 0; i < assignment.size(); i++){
            double temp_radar_time = _obs_track[assignment[i].first].get_obs_radar()->_header._stamp.to_second();

            _obs_track[assignment[i].first].get_obs_radar()->flag_repeat++;
            int temp_flag = _obs_track[assignment[i].first].get_obs_radar()->flag_repeat;
            double temp_stamp = _obs_track[assignment[i].first].get_obs_radar()->_obs_time_stamp;
            Obs3dPtr temp_track = _obs_track[assignment[i].first].get_obs_radar();
            int tmp_id = _obs_track[assignment[i].first].get_obs_radar()->_obs_id;
            float last_x = _obs_track[assignment[i].first].get_obs_radar()->_obs_position._x; //use for velocity replacing
            float last_y = _obs_track[assignment[i].first].get_obs_radar()->_obs_position._y;
            // int temp_object_type = 0;//use for camera updata _object_type
            // if( obs_array.get_obstacles()[assignment[i].second]._sensor_type ==7 ){ 
            //     temp_object_type = _obs_track[assignment[i].first].get_obs_radar()->_object_type;
            // }
            PoseWithCovariance temp_object_box_center = _obs_track[assignment[i].first].get_obs_radar()->object_box_center;
            Vector3 temp_object_box_size = _obs_track[assignment[i].first].get_obs_radar()->object_box_size;
            std::vector<Point> temp_contour_points = _obs_track[assignment[i].first].get_obs_radar()->contour_points;

            if( obs_array.get_obstacles()[assignment[i].second]._sensor_type !=7 || control_mode == 111){
                float temp_obj_type = _obs_track[assignment[i].first].get_obs_radar()->_object_type;
                _obs_track[assignment[i].first].set_obs(*temp_track, obs_array.get_obstacles()[assignment[i].second], sensor_type); //kalman filter
                _obs_track[assignment[i].first].get_obs_radar()->_object_type = temp_obj_type;
            }
            _obs_track[assignment[i].first].get_obs_radar()->flag_repeat = temp_flag;
            _obs_track[assignment[i].first].get_obs_radar()->_obs_id = tmp_id;
            float now_x = _obs_track[assignment[i].first].get_obs_radar()->_obs_position._x; //use for velocity replacing
            float now_y = _obs_track[assignment[i].first].get_obs_radar()->_obs_position._y;

            if( obs_array.get_obstacles()[assignment[i].second]._sensor_type == 7 ){ //use for camera updata _object_type
                _obs_track[assignment[i].first].get_obs_radar()->_object_type = obs_array.get_obstacles()[assignment[i].second]._object_type;
            }

            bool velocity_replace = 1; //if the track has not been updated with front radar, calculate velocity by ourselves
            if(temp_flag > 0 && temp_flag < 3){
                for(int j = 0;j<temp_flag;j++){
                    if (_obs_track[assignment[i].first].get_obs_radar()->_hist_states[j]._sensor_type == 0){
                        velocity_replace = 0;
                    }
                }
            }
            else if(temp_flag >= 3){
                for(int j = 0;j<3;j++){
                    if (_obs_track[assignment[i].first].get_obs_radar()->_hist_states[j]._sensor_type == 0){
                        velocity_replace = 0;
                    }
                }
            }
            if(velocity_replace == 1){
                
                double time_diff = time_stamp - temp_radar_time;
                // std::cout << "ori_vx: " << _obs_track[assignment[i].first].get_obs_radar()->_velocity._x << " replace vx: " << (now_x - last_x)/time_diff << std::endl;
                // _obs_track[assignment[i].first].get_obs_radar()->_velocity._x = (now_x - last_x)/time_diff;
                // _obs_track[assignment[i].first].get_obs_radar()->_velocity._y = (now_y - last_y)/time_diff;
            }

            _obs_track[assignment[i].first].get_obs_radar()->_obs_time_stamp = temp_stamp;
            if(_obs_track[assignment[i].first].get_obs_radar()->_sensor_type != 6){
                _obs_track[assignment[i].first].get_obs_radar()->object_box_center = temp_object_box_center;
                _obs_track[assignment[i].first].get_obs_radar()->object_box_size = temp_object_box_size;
                _obs_track[assignment[i].first].get_obs_radar()->contour_points = temp_contour_points;
            }
            for (int j = 0; j < _obs_track[assignment[i].first].get_obs_radar()->contour_points.size(); j++) //translate together with box
            {
                _obs_track[assignment[i].first].get_obs_radar()->contour_points[j].x = _obs_track[assignment[i].first].get_obs_radar()->contour_points[j].x 
                - _obs_track[assignment[i].first].get_obs_radar()->object_box_center.pose.position.x + _obs_track[assignment[i].first].get_obs_radar()->_obs_position._x;
                _obs_track[assignment[i].first].get_obs_radar()->contour_points[j].y = _obs_track[assignment[i].first].get_obs_radar()->contour_points[j].y 
                - _obs_track[assignment[i].first].get_obs_radar()->object_box_center.pose.position.y + _obs_track[assignment[i].first].get_obs_radar()->_obs_position._y;
            }
            _obs_track[assignment[i].first].get_obs_radar()->object_box_center.pose.position.x = _obs_track[assignment[i].first].get_obs_radar()->_obs_position._x;
            _obs_track[assignment[i].first].get_obs_radar()->object_box_center.pose.position.y = _obs_track[assignment[i].first].get_obs_radar()->_obs_position._y;

            //std::cout << "flag: " << _obs_track[assignment[i].first].get_obs_radar()->flag_repeat << std::endl;
            //     _obs_track[assignment[i].first].compute_fusion_obs();
            //    _obs_track[assignment[i].first].update_track();
            //     ROS_INFO("CALLBACK_FRONT_RADAR_point._x:%f",_obs_track[i].get_obs_radar()->_obs_position._x);
            //     ROS_INFO("CALLBACK_FRONT_RADAR_point._y:%f",_obs_track[i].get_obs_radar()->_obs_position._y);

        }

        // for (size_t i = 0; i < assignment.size(); i++)
        // {
        //     std::cout << "after update: " << std::endl;
        //     std::cout << "assignment[" << i << "]: " << assignment[i].first << " " << assignment[i].second << std::endl;
        //     std::cout << "track assignment.x " << _obs_track[assignment[i].first].get_obs_radar()->_obs_position._x << " obs_assignment.x " << obs_array.get_obstacles()[assignment[i].second]._obs_position._x << std::endl;
        //     std::cout << "track assignment.y " << _obs_track[assignment[i].first].get_obs_radar()->_obs_position._y << " obs_assignment.y " << obs_array.get_obstacles()[assignment[i].second]._obs_position._y << std::endl;
        //     // std::cout << "track assignment.vx " << _obs_track[assignment[i].first].get_obs_radar()->_velocity._x << " obs_assignment.vx " << obs_array.get_obstacles()[assignment[i].second]._velocity._x << std::endl;
        //     // std::cout << "track assignment.vy " << _obs_track[assignment[i].first].get_obs_radar()->_velocity._y << " obs_assignment.vy " << obs_array.get_obstacles()[assignment[i].second]._velocity._y << std::endl;
        // }
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
                    // std::cout<<"array id ";
                    // for (int ii=0;ii<id_arr.size();ii++){
                    // std::cout<<" "<<id_arr[ii];
                    // }
                    // std::cout<<std::endl;
                    float track_x = _obs_track[unassigned_track[i]].get_obs_radar()->_obs_position._x;
                    float track_y = _obs_track[unassigned_track[i]].get_obs_radar()->_obs_position._y;
                    float track_slope = (_obs_track[unassigned_track[i]].get_obs_radar()->_obs_position._x - LRR_F_pos_x) / _obs_track[unassigned_track[i]].get_obs_radar()->_obs_position._y;
                    if(control_mode == -1 && obs_array.get_obstacles()[0]._sensor_type != 7){
                        check_id(_obs_track[unassigned_track[i]].get_obs_radar()->_obs_id); 
                        _obs_track[unassigned_track[i]].set_obs_radar(NULL);
                    }else if(control_mode == 111){  //lidar radar all died
                        check_id(_obs_track[unassigned_track[i]].get_obs_radar()->_obs_id); 
                        _obs_track[unassigned_track[i]].set_obs_radar(NULL);
                    }else{
                        if (obs_array.get_obstacles()[0]._sensor_type == 6){
                            if(control_mode == 0){ //front radar died, delete by lidar
                                if (track_slope > lidar_left_slope || track_slope < lidar_right_slope){
                                    check_id(_obs_track[unassigned_track[i]].get_obs_radar()->_obs_id);
                                    _obs_track[unassigned_track[i]].set_obs_radar(NULL);
                                }
                            }
                            else if(control_mode == 1){ //srr died, delete near by lidar
                                if (track_x < lidar_x_max_dist){
                                    check_id(_obs_track[unassigned_track[i]].get_obs_radar()->_obs_id);
                                    _obs_track[unassigned_track[i]].set_obs_radar(NULL);
                                }
                            }
                            else{
                                if (track_x < lidar_x_max_dist && (track_slope > lidar_left_slope || track_slope < lidar_right_slope)){
                                    check_id(_obs_track[unassigned_track[i]].get_obs_radar()->_obs_id);
                                    _obs_track[unassigned_track[i]].set_obs_radar(NULL);
                                }
                            }
                        }
                        else if (obs_array.get_obstacles()[0]._sensor_type == 0){
                            if(control_mode == 6){ //lidar died, delete by front radar
                                if (track_slope > lidar_left_slope || track_slope < lidar_right_slope){
                                    check_id(_obs_track[unassigned_track[i]].get_obs_radar()->_obs_id);
                                    _obs_track[unassigned_track[i]].set_obs_radar(NULL);
                                }
                            }
                            else{ //control mode is 255 or 1
                                if (track_x > lidar_x_max_dist){
                                    check_id(_obs_track[unassigned_track[i]].get_obs_radar()->_obs_id);
                                    _obs_track[unassigned_track[i]].set_obs_radar(NULL);
                                }
                            }
                        }
                        else if (obs_array.get_obstacles()[0]._sensor_type == 1){
                            if ((track_slope > lidar_right_slope) && (track_y < 0)){
                                check_id(_obs_track[unassigned_track[i]].get_obs_radar()->_obs_id);
                                _obs_track[unassigned_track[i]].set_obs_radar(NULL);
                            }
                        }
                        else if (obs_array.get_obstacles()[0]._sensor_type == 2){
                            if ((track_slope < lidar_left_slope) && (track_y > 0)){
                                check_id(_obs_track[unassigned_track[i]].get_obs_radar()->_obs_id);
                                _obs_track[unassigned_track[i]].set_obs_radar(NULL);
                            }
                        }
                    }
                    
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
        double max_dist_cam_plus = tsinghua::dias::fusion::FUSION_PIXEL_THRES+25;

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
        for (int i = 0; i < unassigned_track_idx.size(); ++i){
            for (int j = i+1; j < unassigned_track_idx.size(); ++j){
                if (track_smallest_position[i] == track_smallest_position[j]){
                    if (_obs_track[unassigned_track[unassigned_track_idx[i]]].get_obs_radar()->flag_repeat > _obs_track[unassigned_track[unassigned_track_idx[j]]].get_obs_radar()->flag_repeat){
                        track_smallest_flag[j]=0;
                    }
                    else{
                        track_smallest_flag[i]=0;
                    }
                }
            }
        }

        for (int i=0; i < unassigned_track_idx.size(); ++i){
            if (track_smallest_flag[i] == 1){
                if(obs_array.get_obstacles()[0]._sensor_type==7){
                    std::cout << "camera mat dist:  " << association_mat[unassigned_track_idx[i]][unassigned_obs_idx[track_smallest_position[i]]] << std::endl;
                    if(_obs_track[unassigned_track[unassigned_track_idx[i]]].get_obs_radar()->flag_repeat<10){  
                        if (association_mat[unassigned_track_idx[i]][unassigned_obs_idx[track_smallest_position[i]]] < max_dist_cam_plus){
                            assignment[assignments_num++] =
                                std::make_pair(unassigned_track[unassigned_track_idx[i]],unassigned_obs[unassigned_obs_idx[track_smallest_position[i]]]);                            
                        }
                    }else{
                        if (association_mat[unassigned_track_idx[i]][unassigned_obs_idx[track_smallest_position[i]]] < max_dist_cam){
                            assignment[assignments_num++] =
                                std::make_pair(unassigned_track[unassigned_track_idx[i]],unassigned_obs[unassigned_obs_idx[track_smallest_position[i]]]);
                        }
                    }
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
                if(track_obs._sensor_type != 7){               
                    double radar_time = track_obs._header._stamp.to_second();
                    double time_diff = time_stamp - radar_time;
                    float x2 = track_obs._obs_position._x + track_obs._velocity._x * time_diff;
                    float y2 = track_obs._obs_position._y + track_obs._velocity._y * time_diff;
                    float theta2 = theta_new - track_obs._course_angle_rad;

                    track_cnt[0] = x2*cosf(theta2)+y2*sinf(theta2);
                    track_cnt[1] = y2*cosf(theta2)-x2*sinf(theta2);
                    track_cnt[2] = track_obs._obs_position._z; //predict track before calculating association mat
                    track_cnt[3] = 0;
                    if(track_obs._sensor_type == 1 || track_obs._sensor_type == 2 || track_obs._sensor_type == 6){
                        track_cnt[1] = track_obs._obs_position._y;
                        track_cnt[0] = track_obs._obs_position._x;                      
                    }
                    track_cnt[1] = track_obs._obs_position._y;
                    track_cnt[0] = track_obs._obs_position._x;

                    double c= 0.66;
                    double omega =(track_cnt[0]-LRR_F_pos_x) *-0.00059703 +track_cnt[1] *-0.000050721 + c*0.00010254-0.0016;
                    unassigned_track_sync.get_obstacles()[i]._v_center= ((track_cnt[0]-LRR_F_pos_x)*-0.22384 + track_cnt[1]* 0.24574 +c*0.045906 -0.5648) / omega ;
                    unassigned_track_sync.get_obstacles()[i]._u_center= ((track_cnt[0]-LRR_F_pos_x) * -0.11899 +track_cnt[1] * -0.026076 +c*0.28690-0.8252) / omega ;   

                    obs_cnt[0] = obs_obs._obs_position._x;
                    obs_cnt[1] = obs_obs._obs_position._y;
                    obs_cnt[2] = obs_obs._obs_position._z;
                    obs_cnt[3] = 0;
                }else{
                    double c= 0.66;
                    double omega =(track_obs._obs_position._x-LRR_F_pos_x) *-0.00059703 +track_obs._obs_position._y *-0.000050721 + c*0.00010254-0.0016;
                    track_cnt[0]= ((track_obs._obs_position._x-LRR_F_pos_x)*-0.22384 + track_obs._obs_position._y* 0.24574 +c*0.045906 -0.5648) / omega ;
                    track_cnt[1]= ((track_obs._obs_position._x-LRR_F_pos_x) * -0.11899 +track_obs._obs_position._y * -0.026076 +c*0.28690-0.8252) / omega ;  
                    // track_cnt[0] = track_obs._v_center;
                    // track_cnt[1] = track_obs._u_center;
                    track_cnt[2] = track_obs._obs_position._x;
                    track_cnt[3] = track_obs._obs_position._y;
                    
                    // std::cout << "track_cnt " << i << ": " << track_cnt[0] << " " << track_cnt[1] << " " << track_cnt[2] << std::endl;
                    obs_cnt[0] = obs_obs._v_center;
                    obs_cnt[1] = obs_obs._u_center;
                    obs_cnt[2] = obs_obs._obs_position._x;
                    obs_cnt[3] = obs_obs._obs_position._y; 
                }

                double diff_pos[4];
                diff_pos[0] = track_cnt[0] - obs_cnt[0];
                diff_pos[1] = track_cnt[1] - obs_cnt[1];
                diff_pos[2] = track_cnt[2] - obs_cnt[2];
                diff_pos[3] = track_cnt[3] - obs_cnt[3];
 
                //diff_pos[2] = track_cnt[2] - obs_cnt[2]; //problem detected: all 3d should be replaced by 2d. jxy 0517
                double w1=1.0,w2= 5.0f;
                double dist_pos = 0.0f;
                if(track_obs._sensor_type != 7){
                    dist_pos = sqrt(pow(diff_pos[0], 2) + pow(diff_pos[1], 2));
                }else{
                    dist_pos = w1*sqrt(pow(diff_pos[0], 2) + pow(diff_pos[1], 2))+w2*sqrt(pow(diff_pos[2], 2) + pow(diff_pos[3], 2));
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

    const std::vector<tsinghua::dias::fusion::OBSFUncertaintyTrack> &get_fusion_result()const{
        return _obs_track;
    }

    tsinghua::dias::fusion::OBSFObs3dFrame& get_lidar_frame() {
        return _lidar_obs_array;
    }

    tsinghua::dias::fusion::OBSFObs3dFrame& get_radar_frame() {
        return _radar_obs_array;
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

};

}//
}//
}//

#endif
