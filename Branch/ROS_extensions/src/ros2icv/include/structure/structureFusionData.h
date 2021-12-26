#ifndef _fusionstruct_H
#define _fusionstruct_H


#include <cstddef>
#include <cstdlib>
#include <string>
#include <sstream>
#include <vector>
#include "OpenICV/structure/header.h"
#include "OpenICV/structure/structureBBox2D.h"

typedef unsigned char uint8;
typedef float float32;
typedef double float64;
typedef char int8;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned int uint32;
typedef int int32;


using namespace std;

/////////////////////////////
  struct BoundingBox
{
float64 probability;
float32 xmin;
float32 ymin;
float32 xmax;
float32 ymax;
float32 x_world;
float32 y_world;
string Class;
MSGPACK_DEFINE(probability,xmin, ymin, xmax, ymax, x_world, y_world, Class);

};
/////////////////////////////////////////

struct ColorRGBA
{

  ColorRGBA()
    : r(0.0)
    , g(0.0)
    , b(0.0)
    , a(0.0)  {
    }




    float  r;
    float  g;
    float  b;
    float  a;
    MSGPACK_DEFINE(r,g,b,a);

}; // struct ColorRGBA_


/////////////////////////////////////////
 struct BoundingBoxes
{
  Header header;
  vector<BoundingBox> boundingBoxes;
  MSGPACK_DEFINE(header, boundingBoxes);
};
/////////////////////////////////////////
 struct Point
{
float64 x;
float64 y;
float64 z;
  Point()
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  MSGPACK_DEFINE(x,y,z);
};
/////////////////////////////////////////
 struct Quaternion
{
float64 x;
float64 y;
float64 z;
float64 w;
  Quaternion()
    : x(0.0)
    , y(0.0)
    , z(0.0) 
    , w(0.0) {
    }
    MSGPACK_DEFINE(x,y,z,w);
};
/////////////////////////////////////////
 struct Vector3
{
float64 x;
float64 y;
float64 z;
  Vector3()
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
    MSGPACK_DEFINE(x,y,z);
};
/////////////////////////////////////////
 struct Pose
{
Point position;
Quaternion orientation;
Pose():
position(),
orientation(){

}
MSGPACK_DEFINE(position, orientation);
};
/////////////////////////////////////////
 struct Twist
{
Vector3 linear;
Vector3 angular;
Twist():
linear(),
angular(){

}
MSGPACK_DEFINE(linear, angular);
};
/////////////////////////////////////////
 struct TwistWithCovariance
{

Header header;
Twist twist;
double covariance[36];
TwistWithCovariance():
twist()
{
}
MSGPACK_DEFINE(header, twist, covariance[36]);
};
/////////////////////////////////////////

 struct PoseWithCovariance
{

Header header;
Pose pose;
double covariance[36];
PoseWithCovariance():
pose()
{
};
MSGPACK_DEFINE(header, pose, covariance[36]);

};
/////////////////////////////////////////
 struct DriveObs
{

int32 obstacle_id;


Point position;
float32 theta;
Vector3 velocity;

float32 length;

float32 width;

float32 height;

vector<Point> polygon_points;


float32 life;


int16 OBSTACLE_TYPE_UNCLASSIFIED = 0  ;          // 0: No classification determined (yet)
int16 OBSTACLE_TYPE_UNKNOWN_SMALL = 1   ;        // 1: Relatively small structure like pole, tree, single bush, etc. which does not fit t
int16 OBSTACLE_TYPE_UNKNOWN_BIG = 2   ;          // 2: Bigger structure which does not fit other classes.
int16 OBSTACLE_TYPE_PEDESTRIAN = 3    ;          // 3: Pedestrian, usually determined by moving behaviour.
int16 OBSTACLE_TYPE_BIKE = 4   ;                 // 4: bike, motor bike
int16 OBSTACLE_TYPE_CAR = 5  ;                   // 5: Passenger car.
int16 OBSTACLE_TYPE_TRUCK = 6 ;                  // 6: Big vehicle, bus, truck.
int16 OBSTACLE_TYPE_FENCE = 7  ;                 // 7: continuous fence on the side of the read
int16 OBSTACLE_TYPE_PLANTS = 8 ;                 // 8: continuous bushs, flowers, etc.

int16 classification;
MSGPACK_DEFINE(obstacle_id, position, theta, velocity, length, width, height, polygon_points, life, OBSTACLE_TYPE_BIKE, OBSTACLE_TYPE_CAR, OBSTACLE_TYPE_FENCE,\
OBSTACLE_TYPE_PEDESTRIAN, OBSTACLE_TYPE_PLANTS, OBSTACLE_TYPE_TRUCK, OBSTACLE_TYPE_UNCLASSIFIED, OBSTACLE_TYPE_UNKNOWN_BIG,OBSTACLE_TYPE_UNKNOWN_SMALL, classification);
};

/////////////////////////////////////////
 struct DriveObsArray
{

   Header header;
   vector<DriveObs> obstacles;
   MSGPACK_DEFINE(header, obstacles);
};
/////////////////////////////////////////
 struct DriveRadarObs
{
int32 obstacle_id;

// Obstacle range to the radar 
// unit = m
float64 range;

// Obstacle azimuth angle to the radar perpendicular bisectors 
// (+) clockwise
// unit = degree
float64 theta;

// Obstacle radial velocity relative to the radar 
// (+) away from the radar
// unit = m/s
float64 radial_velocity;

// Obstacle tangential velocity relative to the radar 
// (+) clockwise
// unit = m/s
float64 tangential_velocity;

// Obstacle radial acceleration relative to the radar 
// (+) away from the radar
// unit = m/s
float64 radial_acceleration;

// Obstacle Radar Cross-Section
// unit = dBsm
int32 rcs  ;

// Obstacle width
// unit = m
float64 width ;
MSGPACK_DEFINE(obstacle_id, range, theta, radial_velocity, tangential_velocity, radial_acceleration, rcs, width);

};

/////////////////////////////////////////
 struct DriveRadarObsArray
{
Header header;
vector<DriveRadarObs>  obstacles;
MSGPACK_DEFINE(header, obstacles);
};
/////////////////////////////////////////

 struct Object
{
int32 id;

int32 tracking_time ;
int32 last_seen;

TwistWithCovariance velocity;

Pose bounding_box_center;
Vector3 bounding_box_size;

PoseWithCovariance object_box_center;
Vector3 object_box_size;
vector<Point> contour_points;
MSGPACK_DEFINE(id, tracking_time, last_seen, velocity, bounding_box_center, bounding_box_size, object_box_center, object_box_size, contour_points);
};

///////////////////////////
 struct ObjectArray
{
Header header;
vector<Object>  objects;
MSGPACK_DEFINE(header, objects);
};

/////////////////////////////////////////
 struct radarlrrobject
{

  uint32 time;
  uint8 id;
  float32 x;
  float32 y;
  float32 speedx;
  float32 speedy;

  float32 accx;
  float32 accy;

  float32 obj_amp;
  uint16 objDynProp;

  float32 rangex_rms;
  float32 rangey_rms;

  float32 speedx_rms;
  float32 speedy_rms;

  float32 accx_rms;
  float32 accy_rms;

  float32 orient_rms;

  uint16 objProbExist;

  uint16 objMeasState;
  int16 objClass;

  float32 ObjectOrientAngel;

  float32 ObjectWidth;
  float32 ObjectLength;
  MSGPACK_DEFINE(time, id, x, y, speedx, speedy, accx, accy, obj_amp, objDynProp, rangex_rms, rangey_rms, speedx_rms, speedy_rms, accx_rms, accy_rms, \
  orient_rms, objProbExist, objMeasState, objClass, ObjectOrientAngel, ObjectWidth, ObjectLength);
};
/////////////////////////////////////////
 struct radarsrrobject
{
uint8 id;
float32 x;
float32 y;
float32 speedx;
float32 speedy;
float32 trackLifeTime;
float32 obj_amp;
uint16 trackIndex;
uint16 trackIndex2;
MSGPACK_DEFINE(id, x, y, speedx, speedy, trackLifeTime, obj_amp, trackIndex, trackIndex2);
};
/////////////////////////////////////////
 struct ivsensorlrrobj
{
  Header header;
  uint32 time;
  vector<radarlrrobject> obs[255];
  MSGPACK_DEFINE(header, time, obs);

};
/////////////////////////////////////////
 struct ivsensorsrrobj
{
  Header header;
  uint32 time;
  vector<radarsrrobject> obs[255];
  MSGPACK_DEFINE(header, time, obs);
};
/////////////////////////////////////////
struct track
{

  uint32 time;
  uint8 id;
  uint8 flag_repeat;
  uint8 object_type;
  float32 course_angle_rad;
  Point object_box_center;
  Point object_box_size;
  vector<Point> contour_points;
  Point obs_position ;
  Point velocity ;
  Point acceleration ;
  MSGPACK_DEFINE(time, id, flag_repeat, object_type, course_angle_rad, object_box_center, object_box_size, contour_points, obs_position, velocity);
};
struct TrackArray
{

Header header;
vector<track> tracks;
Point self_velocity;
Point self_acceleration;
Point gps_route;
Point imu_pose ;
MSGPACK_DEFINE(header, tracks, self_velocity, self_acceleration, imu_pose, gps_route);
};






////////////////////////////////////////
struct Marker
{


  Marker()
  : 
    id(0)
  , type(0)
  , action(0)
  , pose()
  , scale()
  , color()
  , lifetime()
  , frame_locked(false)
  , points()
  , text()
  , mesh_resource()
  , mesh_use_embedded_materials(false)  {
  };
  Header header;

  int32_t  id;
  int32_t  type;
  int32_t  action;
  string ns;
  Pose pose;
  Vector3 scale;
  ColorRGBA color;
  uint32 lifetime;
  uint8_t  frame_locked;
  vector<Point>  points;
  vector<ColorRGBA> colors;
  string text;
  string mesh_resource;
  uint8_t  mesh_use_embedded_materials;
  MSGPACK_DEFINE(header, id, type, action, ns, pose, scale, color, lifetime, frame_locked, points, colors, text, mesh_resource, mesh_use_embedded_materials);



  enum {
    ARROW = 0u,
    CUBE = 1u,
    SPHERE = 2u,
    CYLINDER = 3u,
    LINE_STRIP = 4u,
    LINE_LIST = 5u,
    CUBE_LIST = 6u,
    SPHERE_LIST = 7u,
    POINTS = 8u,
    TEXT_VIEW_FACING = 9u,
    MESH_RESOURCE = 10u,
    TRIANGLE_LIST = 11u,
    ADD = 0u,
    MODIFY = 0u,
    DELETE = 2u,
    DELETEALL = 3u,
  };  

}; // struct Marker_


struct MarkerArray
{
Header header;
  vector<Marker> markers;

MSGPACK_DEFINE(header, markers);
};

#endif // 



































