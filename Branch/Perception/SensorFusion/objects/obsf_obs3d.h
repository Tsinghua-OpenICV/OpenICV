#ifndef OBSF_OBS3D_H
#define OBSF_OBS3D_H

#include <vector>
#include "obsf_obs_vector.h"
#include "obsf_header.h"
#include <eigen3/Eigen/Core>   //TODO: get rid of the dependency.
#include "OpenICV/structure/structureIMU.h"
#include "OpenICV/structure/structureFusionData.h"
#include <deque>
typedef unsigned char uint8;
typedef float float32;
typedef double float64;
typedef char int8;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned int uint32;
typedef int int32;

namespace tsinghua{
namespace dias{
namespace fusion{

struct hist_track_obs{
    double _header;
    OBFSVector3d _obs_position;
    OBFSVector3d _velocity;
    float rangex_rms;
    float rangey_rms;
    float speedx_rms;
    float speedy_rms;
    Eigen::Matrix4f P;
    int _sensor_type;
};

class OBSFObs3d{
public:
    OBSFObs3d(){}
    ~OBSFObs3d(){}
   
public:
    OBSFHeader _header;
Header _sensor_header;
    int _obs_id;
    int flag_repeat;
    double _obs_time_stamp;
    //int _flag_too_near;
    float rangex_rms;
    float rangey_rms;
    float speedx_rms;
    float speedy_rms;
    float rangez_rms;
    float speedz_rms;
    float accx_rms;
    float accy_rms;
    double _u_center;
    double _v_center;
    double _width_pixel;
    double _height_pixel;
    double _u_proj;
    double _v_proj;
    int objProbExist;
    int objDynProp;
    int objMeasState;
    int objClass;
    float obj_amp;
    int _sensor_type;// 0:front, 1:right front, 2:left front, 3:rear ,4 ringt rear,　５，left rear, 6:front 4lidar, 7 camera
    int _object_type;//0x0:point, 0x1:car, 0x2:truck, 0x3:pedestrian, 0x4:motorcycle, 0x5:bicycle, 0x6:wide, 0x7:reserved 
    PoseWithCovariance object_box_center;
    Vector3 object_box_size;
    std::vector<Point> contour_points;
    // geometry_msgs::TwistWithCovarianceStamped twist_linear;
    OBFSVector3d _obs_position; //
    OBFSVector3d _obs_local_position; // 
    float _range; //
    float _radial_velocity; //
    float _relative_radial_velocity; // from the output of radar
    float _angle;// 
    float _obs_theta;
    OBFSVector3d _velocity;             // 
    OBFSVector3d _relative_velocity;    //
    OBFSVector3d _self_velocity;
    float _course_angle_rad;
    OBFSVector3d _gps_route;
    float _length;
    float _width;
    float _height;
    std::vector<OBFSVector3d>  _polygon_points; // 
    float _life;
    short _classification;
    std::vector<OBFSVector3d> _cloud_points;  
    Eigen::Matrix4f P; // the uncertainty matrix. 
    std::deque<hist_track_obs> _hist_obs;
    std::deque<hist_track_obs> _hist_states;

   //
#if 0
    OBFSVector3d _obs_position_raw; // in global frame, for visualization of raw output ？
    OBFSVector3d _obs_local_position_raw; // in main car's frame, for visualization of raw output ？    
    OBFSVector3d _obs_previous_position;// ？    
    double rcs; // test for radar visualization ？
#endif
};

// class OBSFObs3d_hist{
// public:
//     OBSFObs3d_hist(){}
//     ~OBSFObs3d_hist(){}
    
// public:
//     OBSFHeader _header;
//     OBFSVector3d _obs_position;
//     OBFSVector3d _velocity;
//     Eigen::Matrix4f P;
// };

class OBSFObs3dTruncated{
public:
    OBSFObs3dTruncated(){}
    ~OBSFObs3dTruncated(){}
    
public:
    OBSFHeader _header;
    int _obs_id;
    OBFSVector3d _obs_position;
    float _obs_theta;
    OBFSVector3d _velocity;
    float _length;
    float _width;
    float _height;
    std::vector<OBFSVector3d>  _polygon_points;
    float _life;
    short _classification;
    float _truncated;
};

}//
}//
}//

#endif
