// @brief: fusion track

// objective: define, observe, update the states of the objects
//Author: Chunlei YU. Contact me at: yuchunlei@mail.tsinghua.edu.cn


#ifndef FUSION_TRACK_H
#define FUSION_TRACK_H

#include <memory>
#include <deque>
#include <iostream>
#include "obsf_obs2d.h"
#include "obsf_obs3d.h"
#include "obsf_header.h"
#include <boost/thread/thread.hpp>
#include "../filters/base_filter_wrap.h"
//#include "../filters/kalman_filter_wrap.h"

#include "../filters/fusion_kalman_filter.h"

const int hist_size = 10;

namespace tsinghua{
namespace dias {
namespace fusion{

class OBSFUncertaintyTrack{
public:
    typedef std::shared_ptr<OBSFObs3d> Obs3dPtr;
    typedef std::shared_ptr<OBSFObs2d> Obs2dPtr;

    OBSFUncertaintyTrack(){
        _s_current_idx %= MAX_IDX;
        _obs_id = _s_current_idx++;

        _obs_2d_camera = NULL;
        _obs_camera = NULL;
        _obs_lidar = NULL;
        _obs_radar = NULL;
        _obs = NULL;

	    //tracked_times = 1;
    }

    template <typename OBSType>
    OBSFUncertaintyTrack(OBSType &obs, SensorType sensor_type){
        _s_current_idx %= MAX_IDX;
        _obs_id = _s_current_idx++;

        _obs_2d_camera = NULL;
        _obs_camera = NULL;
        _obs_lidar = NULL;
        _obs_radar = NULL;
        _obs = NULL;

        _obs = std::shared_ptr<OBSType>(new OBSType());
        if (sensor_type == LIDAR){
            _obs_lidar = std::shared_ptr<OBSType>(new OBSType());
            *_obs_lidar = obs;
            *_obs = obs;
        } else if (sensor_type == CAMERA3D){
            _obs_camera = std::shared_ptr<OBSType>(new OBSType());
            *_obs_camera = obs;
            *_obs = obs;
        } else if (sensor_type == RADAR) {
            _obs_radar = std::shared_ptr<OBSType>(new OBSType());
            *_obs_radar = obs;
            *_obs = obs;
        } else {
            std::cout << "Class OBSFTrack Unknown SensorType!" << std::endl;
        }

        _tracker = boost::shared_ptr<tsinghua::dias::fusion::BaseFilterWrap>
                    (new tsinghua::dias::fusion::FusionKalmanFilter());
        // _tracker->initialize(obs);     


        _hist_obs_3d.push_front(obs);
        _hist_states_3d.push_front(obs);

       // newRadar = false;
       // newLidar = false;
       // newCamera = false;       

	    //tracked_times = 1;//
        //_obs->_obs_id = _obs_id;

        //std::cout << "_obs_id" << _obs_id << std::endl;
        //getchar();
    }

    OBSFUncertaintyTrack(const OBSFUncertaintyTrack &track)
    {
        _obs_id = track._obs_id;

        _obs_2d_camera = track._obs_2d_camera;
        _obs_camera = track._obs_camera;
        _obs_lidar = track._obs_lidar;
        _obs_radar = track._obs_radar;
        _obs = track._obs;
        _tracker = track._tracker;

        _hist_obs_3d = track._hist_obs_3d;
        _hist_states_3d = track._hist_states_3d;


	   // tracked_times = track.tracked_times;  //
    }

    OBSFUncertaintyTrack& operator = (const OBSFUncertaintyTrack &track)
    {
        _obs_id = track._obs_id;

        _obs_2d_camera = track._obs_2d_camera;
        _obs_camera = track._obs_camera;
        _obs_lidar = track._obs_lidar;
        _obs_radar = track._obs_radar;
        _obs = track._obs;
        _tracker = track._tracker;

        _hist_obs_3d = track._hist_obs_3d;
        _hist_states_3d = track._hist_states_3d;

	   // tracked_times = track.tracked_times;  //
        return *this;
    }

    // increse tracked times
    // void increase_tracked_times(){
	//     tracked_times++;
    // }

    // int get_tracked_times(){
	//     return tracked_times;
    // }

    ~OBSFUncertaintyTrack(){}

    int get_obs_id() const{
        return _obs_id;
    }

    Obs2dPtr get_obs2d_camera(){
        return _obs_2d_camera;
    }

    Obs3dPtr get_obs_camera(){
        return _obs_camera;
    }

    const Obs3dPtr get_obs_camera()const {
        return _obs_camera;
    }

    void set_obs_camera(Obs3dPtr obs_camera){
        _obs_camera = obs_camera;
    }

    Obs3dPtr get_obs_lidar(){
        return _obs_lidar;
    }

    const Obs3dPtr get_obs_lidar()const {
        return _obs_lidar;
    }

    void set_obs_lidar(Obs3dPtr obs_lidar){
        _obs_lidar = obs_lidar;
    }

    Obs3dPtr get_obs_radar() {
        return _obs_radar;
    }

    const Obs3dPtr get_obs_radar() const {
        return _obs_radar;
    }

    void set_obs_radar(Obs3dPtr obs_radar) {
        _obs_radar = obs_radar;
    }

    Obs3dPtr get_obs(){
        return _obs;
    }

    const Obs3dPtr get_obs()const {
        return _obs;
    }

    // object_time should be seconds.
    void prediction(const tsinghua::dias::fusion::OBSFHeader &header){    //double object_time
        double time_stamp = _obs->_header._stamp.to_second();
        double time_diff = header._stamp.to_second() - time_stamp;
        if(time_diff < 0)
            std::cerr << "error, new objects younger than track age" << std::endl;
        else{
             Eigen::Vector4f s = _tracker->predict(time_diff);

            (*_obs)._obs_position._x = s[0];
            (*_obs)._obs_position._y = s[1];
            (*_obs)._velocity._x = s[2];
            (*_obs)._velocity._y = s[3];     
            //(*_obs)._header = header;

            (*_obs).P = _tracker->getCovarianceMatrix();

        }
           
    }

// whenever a new observation arrives.
    template <typename OBSType>
    void set_obs(OBSType &track_raw, OBSType &ob, const SensorType &sensor_type)
    {

        double observation_time = ob._header._stamp.to_second();
        double track_time = _obs->_header._stamp.to_second();

        double diff = observation_time - track_time;

        #if 0
            if (diff < 0) diff += 0.1;
        #endif

        if(diff >= 0){
            // observation is new, then update the track normally
            _tracker->predict(diff);
            Eigen::Vector4f s;
            if (sensor_type == LIDAR){
                _obs_lidar = std::shared_ptr<OBSType>(new OBSType);
                *_obs_lidar = ob;

                s = _tracker->update_with_object(*_obs_lidar);
                *_obs = *_obs_lidar;
            } else if (sensor_type == CAMERA3D){
                _obs_camera = std::shared_ptr<OBSType>(new OBSType);
                *_obs_camera = ob;

                s = _tracker->update_with_object(*_obs_camera);
                *_obs = *_obs_camera;
            } else if (sensor_type == RADAR) {
                _sensor_track = std::shared_ptr<OBSType>(new OBSType);
                *_sensor_track = track_raw;//_sensor_track is history track.

                _obs_radar = std::shared_ptr<OBSType>(new OBSType);
                *_obs_radar = ob; //_obs_radar is new.
                
                _tracker->initialize(*_sensor_track);
                s = _tracker->update_with_object(*_obs_radar); //s includes old information.
                *_obs = *_obs_radar;
            } else {
                std::cout << "Unknow SensorType!" << std::endl;
            }

            // (*_obs_radar)._obs_position._x = s[0];
            // (*_obs_radar)._obs_position._y = s[1];
            // (*_obs_radar)._velocity._x = s[2];
            // (*_obs_radar)._velocity._y = s[3];
            // (*_obs_radar).P = _tracker->getCovarianceMatrix();

            (*_obs)._obs_position._x = s[0];
            (*_obs)._obs_position._y = s[1];
            (*_obs)._velocity._x = s[2];
            (*_obs)._velocity._y = s[3];
            (*_obs).P = _tracker->getCovarianceMatrix();   

            temp_hist_track._header = _obs->_header._stamp.to_second();
            temp_hist_track._obs_position._x = s[0];
            temp_hist_track._obs_position._y = s[1];
            temp_hist_track._velocity._x = s[2];
            temp_hist_track._velocity._y = s[3];
            temp_hist_track.P = _tracker->getCovarianceMatrix();
            temp_hist_track._sensor_type = _obs->_sensor_type;
            track_raw._hist_states.push_front(temp_hist_track);   // store the history in hist_states;
            if(track_raw._hist_states.size() > hist_size) track_raw._hist_states.pop_back();

            temp_hist_obs._header = ob._header._stamp.to_second();
            temp_hist_obs._obs_position._x = ob._obs_position._x;
            temp_hist_obs._obs_position._y = ob._obs_position._y;
            temp_hist_obs._velocity._x = ob._velocity._x;
            temp_hist_obs._velocity._y = ob._velocity._y;
            temp_hist_obs.rangex_rms = ob.rangex_rms;                
            temp_hist_obs.speedx_rms = ob.speedx_rms;                
            temp_hist_obs.rangey_rms = ob.rangey_rms;
            temp_hist_obs.speedy_rms = ob.speedy_rms;
            track_raw._hist_obs.push_front(temp_hist_obs); // store the history in hist_obs;
            if(track_raw._hist_obs.size() > hist_size) track_raw._hist_obs.pop_back();
            
            // history length is kept no larger than 10
            // TODO change this hard code.


            if (sensor_type == LIDAR){
                *_obs_lidar = *_obs;
            } else if (sensor_type == CAMERA3D){
                *_obs_camera = *_obs;
            } else if (sensor_type == RADAR) {
                *_obs_radar = *_obs;
            } else {
                std::cout << "Unknow SensorType!" << std::endl;
            }  
            

        }else{
            std::cerr << "error, new objects younger than track age" << std::endl;
            // std::cout << "observation time is " << observation_time << std::endl;
            // std::cout << "track time is " << track_time << std::endl;         

            // the observation is delayed. need special treatment
            // firstly, find the first state (X) that is younger than the observation 
            int id = 0;
            for(id = 0; id < track_raw._hist_states.size(); id++){
                if(track_raw._hist_states[id]._header < observation_time)
                    break;
            }
            // this state[id], should be updated by this new observation.
            // confirm the existence of this state
            if(id >= track_raw._hist_states.size()) //  no such state exists. Then this new observation is not integrated.
                return;      
            // such state exists.
            {
                Eigen::Vector4f s;
                boost::shared_ptr<tsinghua::dias::fusion::BaseFilterWrap>   _temp_tracker;  
                _temp_tracker = boost::shared_ptr<tsinghua::dias::fusion::BaseFilterWrap>
                            (new tsinghua::dias::fusion::FusionKalmanFilter()); 
                // initialized by the track_raw._hist_states[id]...                
                double time_diff = observation_time - track_raw._hist_states[id]._header;
                //time_diff = obs(observation_time - _hist_states[id]._header); 
                _temp_tracker->predict_hist(time_diff);
                _temp_tracker->initialize_hist(track_raw._hist_states[id]);
                s = _temp_tracker->update_with_object(ob);
                // begin the update process. This process iterates from the ob._hist_obs[id] to the youngeset observation stored in ob._hist_obs[id]
                double previous_time = observation_time;
                for(id -= 1; id >=0; id--){
                    double diff = track_raw._hist_obs[id]._header - previous_time;
                    _temp_tracker->predict_hist(diff);
                    // _temp_tracker->initialize_hist(track_raw._hist_states[id]);
                    s = _temp_tracker->update_with_object_hist(track_raw._hist_obs[id]);

                    // update the track._hist_states
                    track_raw._hist_states[id]._obs_position._x = s[0];
                    track_raw._hist_states[id]._obs_position._y = s[1];
                    track_raw._hist_states[id]._velocity._x = s[2];
                    track_raw._hist_states[id]._velocity._y = s[3];  
                    track_raw._hist_states[id].P = _temp_tracker->getCovarianceMatrix();

                    previous_time = track_raw._hist_obs[id]._header;
                }

                (*_obs)._obs_position._x = s[0];
                (*_obs)._obs_position._y = s[1];
                (*_obs)._velocity._x = s[2];
                (*_obs)._velocity._y = s[3];  

                (*_obs).P = _temp_tracker->getCovarianceMatrix();       

                // 恢复现场
                _tracker.swap(_temp_tracker);          
            }

            if (sensor_type == LIDAR){
                *_obs_lidar = *_obs;
            } else if (sensor_type == CAMERA3D){
                *_obs_camera = *_obs;
            } else if (sensor_type == RADAR) {
                *_obs_radar = *_obs;
            } else {
                std::cout << "Unknow SensorType!" << std::endl;
            } 
            // update the obs.. especially the header (used in the delete process of respective tracks)
            // if (sensor_type == LIDAR){
            //     _obs_lidar = std::shared_ptr<OBSType>(new OBSType);
            //     *_obs_lidar = ob;
            // } else if (sensor_type == CAMERA3D){
            //     _obs_camera = std::shared_ptr<OBSType>(new OBSType);
            //     *_obs_camera = ob;
            // } else if (sensor_type == RADAR) {
            //     _obs_radar = std::shared_ptr<OBSType>(new OBSType);
            //     *_obs_radar = ob;
            // } else {
            //     std::cout << "Unknow SensorType!" << std::endl;
            // }            

        }

    }


    // update track according to the new observation. 
    void update_track(){
            //Eigen::Vector4f s = _tracker->update_with_object(*_obs_radar);
            Eigen::Vector4f s;
            //bool check = _tracker->update_with_object(*_obs_radar, s, id_tracked);
#if 0
            if(newRadar){
                s = _tracker->update_with_object(*_obs_radar);
                newRadar = false;
                *_obs = *_obs_radar;
               
            }else if(newLidar){
                s = _tracker->update_with_object(*_obs_lidar);
                newLidar = false;
                *_obs = *_obs_lidar;
            }else if(newCamera){
                s = _tracker->update_with_object(*_obs_camera);
                newCamera = false;
                *_obs = *_obs_camera;
            }else {
                std::cout << "No new observation, can not update fusion result!" << std::endl;
            }
#endif
            (*_obs)._obs_position._x = s[0];
            (*_obs)._obs_position._y = s[1];
            (*_obs)._velocity._x = s[2];
            (*_obs)._velocity._y = s[3];  

            (*_obs).P = _tracker->getCovarianceMatrix(); 
          
    }

    void compute_fusion_obs(){
    /*    
        if (_obs_lidar != NULL && _obs_camera != NULL){
            //std::cout << "compute_fusion_obs_not_null" << std::endl;
            idl::car::obsf::CommonUtil::compute_camera_lidar_fusion_property(
                *_obs_lidar, *_obs_camera, *_obs);
        } else if (_obs_lidar != NULL && _obs_radar != NULL) {
            idl::car::obsf::CommonUtil::compute_lidar_radar_fusion_property(
                *_obs_lidar, *_obs_radar, *_obs);
        } else if (_obs_radar !=NULL && _obs_camera != NULL) {
            idl::car::obsf::CommonUtil::compute_radar_camera_fusion_property(
                *_obs_radar, *_obs_camera, *_obs);
        } else {
            if (_obs_lidar !=NULL) {
                *_obs = *_obs_lidar;
            } else if (_obs_camera != NULL){
                *_obs = *_obs_camera;
            } else if (_obs_radar != NULL) {
                *_obs = *_obs_radar;
            } else {
                std::cerr << "Error Track!" << std::endl;
            }
        }
    */    
    }
    static int _s_current_idx;
private:
    int                    _obs_id;

    Obs2dPtr               _obs_2d_camera;
    Obs3dPtr               _obs_camera;

    Obs3dPtr               _obs_lidar;

    Obs3dPtr               _obs_radar;
    
    Obs3dPtr               _sensor_track;

    Obs3dPtr               _obs;

    boost::shared_ptr<tsinghua::dias::fusion::BaseFilterWrap>    _tracker;
   
    tsinghua::dias::fusion::hist_track_obs temp_hist_obs;
    tsinghua::dias::fusion::hist_track_obs temp_hist_track;
    //bool newRadar;
    //bool newLidar;
    //bool newCamera;

    std::deque<tsinghua::dias::fusion::OBSFObs3d> _hist_obs_3d;
    std::deque<tsinghua::dias::fusion::OBSFObs3d> _hist_states_3d;

    //int tracked_times; //test
};

typedef std::map<int, OBSFUncertaintyTrack> OBSFUncertaintyMapInt2Track;

class OBSFUncertaintyTrackSet{
public:
    OBSFUncertaintyTrackSet();
    ~OBSFUncertaintyTrackSet();

    bool add_track(OBSFUncertaintyTrack& track_item);

    OBSFUncertaintyTrack* get_track_item(int obs_id);
    const OBSFUncertaintyTrack* get_track_item(int obs_id) const;
    void remove_track_item(int obs_id);
    void release();
    int get_track_number() const;
private:

    /*map from global obs id to track item */
    OBSFUncertaintyMapInt2Track    _tracks;
};

}//
}
}

#endif
