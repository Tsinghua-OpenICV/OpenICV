#ifndef OBJECTS_OBSF_TRACK_H
#define OBJECTS_OBSF_TRACK_H

#include <memory>
#include <iostream>
#include "obsf_define.h"
#include "obsf_obs2d.h"
#include "obsf_obs3d.h"
//#include "obsf_obs3d_lidar.h"
//#include "common_util.h"

namespace tsinghua{
namespace dias{
namespace fusion{

class OBSFTrack{
public:
    typedef std::shared_ptr<OBSFObs3d> Obs3dPtr;
    typedef std::shared_ptr<OBSFObs2d> Obs2dPtr;

    OBSFTrack(){
        _s_current_idx %= MAX_IDX;
        _obs_id = _s_current_idx++;

        _obs_2d_camera = NULL;
        _obs_camera = NULL;
        //_obs_lidar = NULL;
        _obs_radar = NULL;
        _obs = NULL;

	tracked_times = 1;
    }

    template <typename OBSType>
    OBSFTrack(OBSType &obs, SensorType sensor_type){
        _s_current_idx %= MAX_IDX;
        _obs_id = _s_current_idx++;

        _obs_2d_camera = NULL;
        _obs_camera = NULL;
        //_obs_lidar = NULL;
        _obs_radar = NULL;
        _obs = NULL;

        _obs = std::shared_ptr<OBSType>(new OBSType());
        if (sensor_type == LIDAR){
            //_obs_lidar = std::shared_ptr<OBSType>(new OBSType());
            //*_obs_lidar = obs;
            //*_obs = obs;
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
	tracked_times = 1;//
        //_obs->_obs_id = _obs_id;

        //std::cout << "_obs_id" << _obs_id << std::endl;
        //getchar();
    }

    OBSFTrack(const OBSFTrack &track)
    {
        _obs_id = track._obs_id;

        _obs_2d_camera = track._obs_2d_camera;
        _obs_camera = track._obs_camera;
        //_obs_lidar = track._obs_lidar;
        _obs_radar = track._obs_radar;
        _obs = track._obs;
	tracked_times = track.tracked_times;  //
    }

    OBSFTrack& operator = (const OBSFTrack &track)
    {
        _obs_id = track._obs_id;

        _obs_2d_camera = track._obs_2d_camera;
        _obs_camera = track._obs_camera;
        //_obs_lidar = track._obs_lidar;
        _obs_radar = track._obs_radar;
        _obs = track._obs;

	tracked_times = track.tracked_times;  //
        return *this;
    }

    // increse tracked times
    void increase_tracked_times()
   {
	tracked_times++;
   }

   int get_tracked_times(){
	return tracked_times;
   }

    ~OBSFTrack(){}

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

    template <typename OBSType>
    void set_obs(OBSType &ob, const SensorType &sensor_type){
        if (sensor_type == LIDAR){
            // _obs_lidar = std::shared_ptr<OBSType>(new OBSType);
            // *_obs_lidar = ob;
        } else if (sensor_type == CAMERA3D){
            _obs_camera = std::shared_ptr<OBSType>(new OBSType);
            *_obs_camera = ob;
        } else if (sensor_type == RADAR) {
            _obs_radar = std::shared_ptr<OBSType>(new OBSType);
            *_obs_radar = ob;
        } else {
            std::cout << "Unknow SensorType!" << std::endl;
        }
    }
    void compute_fusion_obs(){
        //std::cout << "compute_fusion_obs" << std::endl;
        if (_obs_lidar != NULL && _obs_camera != NULL){
            //std::cout << "compute_fusion_obs_not_null" << std::endl;
            //obsf::CommonUtil::compute_camera_lidar_fusion_property(
                //*_obs_lidar, *_obs_camera, *_obs);
        } else if (_obs_lidar != NULL && _obs_radar != NULL) {
            //obsf::CommonUtil::compute_lidar_radar_fusion_property(
                //*_obs_lidar, *_obs_radar, *_obs);
        } else if (_obs_radar !=NULL && _obs_camera != NULL) {
            //obsf::CommonUtil::compute_radar_camera_fusion_property(
                //*_obs_radar, *_obs_camera, *_obs);
        }else {
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
        //_obs->_obs_id = _obs_id;   //all have a new id
    }
    static int _s_current_idx;
private:
    int                    _obs_id;

    Obs2dPtr               _obs_2d_camera;
    Obs3dPtr               _obs_camera;

    Obs3dPtr               _obs_lidar;

    Obs3dPtr               _obs_radar;

    Obs3dPtr               _obs;

    int tracked_times; //test
};

typedef std::map<int, OBSFTrack> OBSFMapInt2Track;

class OBSFTrackSet{
public:
    OBSFTrackSet();
    ~OBSFTrackSet();

    bool add_track(OBSFTrack& track_item);

    OBSFTrack* get_track_item(int obs_id);
    const OBSFTrack* get_track_item(int obs_id) const;
    void remove_track_item(int obs_id);
    void release();
    int get_track_number() const;
private:

    /*map from global obs id to track item */
    OBSFMapInt2Track    _tracks;
};

}//
}//
}//

#endif
