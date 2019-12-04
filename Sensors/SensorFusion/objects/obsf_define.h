#ifndef OBSF_DEFINE_H
#define OBSF_DEFINE_H

#include <vector>
#include <map>


namespace tsinghua{
namespace dias {
namespace fusion{

typedef double                                                 OBSFFloatType;
typedef float                                                  OBSFRealValuedType;

enum SensorType {
    UNKNOWN = 0,
    LIDAR = 1,
    CAMERA2D  = 2,
    CAMERA3D  = 3,
    RADAR = 4
};
const OBSFFloatType FUSION_THRES    =                          1.0; //2.5
const OBSFFloatType FUSION_PIXEL_THRES =                       45.0;
const OBSFFloatType LIDAR_TIME_WIN  =                          0.25;
const OBSFFloatType CAMERA_TIME_WIN =                          0.25;
const OBSFFloatType RADAR_TIME_WIN  =                          0.3;
const int           MAX_IDX         =                          2147483647;
extern bool DEBUG;
extern bool OBSF_VISUAL;
extern bool USE_PROTO_OR_NOT;
extern bool use_had_map;
// bool                DEBUG           =                          true;
// bool                OBSF_VISUAL     =                          true;
const double        PI              =                          3.1415926;
const double        max_percep_dist =                          60.0;
const double        xoffset         =                          440335;    //    456000
const double        yoffset         =                          4429710;    //   4390000
const double        zoffset         =                          41;

const double        radar_cycle     =                          0.05;

const double        radar_range_thresh    =                          0.1;
const double        radar_velocity_thresh =                          0.0;
const int           radar_power_thresh    =                          -10;

} //
} //
} //

#endif
