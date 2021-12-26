

#ifndef _imustruct_H
#define _imustruct_H
#include <stdint.h>
#include "time.h" 
#include <string>
#include <msgpack.hpp>


struct twist_
{
  struct twist__
  {
    struct linear__{
      double x;
      double y;
      double z;
      MSGPACK_DEFINE( x, y,z);

    }linear;
        MSGPACK_DEFINE( linear);

  }twist;
  double covariance[36];
  MSGPACK_DEFINE( twist, covariance);

};

struct NavSatFix
{
  struct header_
  {
    time_t stamp; 
    char frame_id[16];
    MSGPACK_DEFINE( stamp, frame_id);

  }header;
  double latitude;
  double longitude;
  double altitude;
  int32_t heading ; // 1e-6 rad
  uint8_t position_covariance_type;
  struct status_
  {
    uint8_t status;
    uint8_t service;
          MSGPACK_DEFINE( status, service);

  }status;

  double position_covariance[9];

   MSGPACK_DEFINE(header,latitude,longitude,altitude,position_covariance_type,status,position_covariance);

};

struct TwistWithCovarianceStamped
{
  NavSatFix::header_ header;
  twist_ twist;
  MSGPACK_DEFINE(header,twist);

};
struct linear_acceleration_
{
  double x;
  double y;
  double z;
  MSGPACK_DEFINE(x,y,z);
};
struct angular_velocity_
{
  double x;
  double y;
  double z;
  MSGPACK_DEFINE( x, y,z);
};

struct orientation_
{
  double x;
  double y;
  double z;
  double w;
  MSGPACK_DEFINE( x, y,z,w);

};


struct Imu 
{
    NavSatFix::header_ header;
    struct linear_acceleration_ linear_acceleration;
    struct angular_velocity_ angular_velocity;
    struct orientation_ orientation;

    double orientation_covariance[9];
    double angular_velocity_covariance[9];
    double linear_acceleration_covariance[9];

    MSGPACK_DEFINE( header, linear_acceleration,angular_velocity,orientation,orientation_covariance,angular_velocity_covariance,linear_acceleration_covariance);

};


struct Odometry
{
  NavSatFix::header_ header;
  char child_frame_id[16];
  twist_ twist;
  MSGPACK_DEFINE(header,child_frame_id,twist);

};
///////////////////////////////////////////
enum {
    STATUS_NO_FIX = -1,
    STATUS_FIX = 0,
    STATUS_SBAS_FIX = 1,
    STATUS_GBAS_FIX = 2,
    SERVICE_GPS = 1u,
    SERVICE_GLONASS = 2u,
    SERVICE_COMPASS = 4u,
    SERVICE_GALILEO = 8u,
  };
///////////////////////////////////////////
  enum {
    COVARIANCE_TYPE_UNKNOWN = 0u,
    COVARIANCE_TYPE_APPROXIMATED = 1u,
    COVARIANCE_TYPE_DIAGONAL_KNOWN = 2u,
    COVARIANCE_TYPE_KNOWN = 3u,
  };



///////////////////////////////////////////
typedef union {
  uint8_t bytes[8];
  struct {
    uint32_t :32;
    uint8_t num_sats;
    uint8_t position_mode;
    uint8_t velocity_mode;
    uint8_t orientation_mode;
  } chan0;
  struct {
    uint16_t acc_position_north; // 1e-3 m
    uint16_t acc_position_east;
    uint16_t acc_position_down;
    uint8_t age;
  } chan3;
  struct {
    uint16_t acc_velocity_north; // 1e-3 m/s
    uint16_t acc_velocity_east;
    uint16_t acc_velocity_down;
    uint8_t age;
  } chan4;
  struct {
    uint16_t acc_heading; // 1e-5 rad
    uint16_t acc_pitch;
    uint16_t acc_roll;
    uint8_t age;
  } chan5;
  struct {
    int16_t base_station_age;
    int8_t base_station_id[4];
  } chan20;
  struct {
    uint16_t delay_ms;
  } chan23;
  struct {
    uint8_t heading_quality; // 0:None 1:Poor 2:RTK Float 3:RTK Integer
  } chan27;
  struct {
    int16_t heading_misalignment_angle; // 1e-4 rad
    uint16_t heading_misalignment_accuracy; // 1e-4 rad
    uint16_t :16;
    uint8_t valid;
  } chan37;
  struct {
    int16_t undulation; // 5e-3 m
    uint8_t HDOP; // 1e-1
    uint8_t PDOP; // 1e-1
  } chan48;
} Channel;


///////////////////////////////////////////
typedef struct {
  uint8_t sync;
  uint16_t time;
  int32_t accel_x ; // 1e-4 m/s^2
  int32_t accel_y ; // 1e-4 m/s^2
  int32_t accel_z ; // 1e-4 m/s^2
  int32_t gyro_x ; // 1e-5 rad/s
  int32_t gyro_y ; // 1e-5 rad/s
  int32_t gyro_z ; // 1e-6 rad/s
  uint8_t nav_status;
  uint8_t chksum1;
  double latitude;
  double longitude;
  float altitude;
  int32_t vel_north ; // 1e-4 m/s
  int32_t vel_east; // 1e-4 m/s
  int32_t vel_down ; // 1e-4 m/s
  int32_t heading ; // 1e-6 rad
  int32_t pitch ; // 1e-6 rad
  int32_t roll ; // 1e-6 rad
  uint8_t chksum2;
  uint8_t channel;
  Channel chan;
  uint8_t chksum3;
} Imu_Packet;



enum {
  MODE_NONE = 0,
  MODE_SEARCH = 1,
  MODE_DOPLER = 2,
  MODE_SPS = 3,
  MODE_DIFFERENTIAL = 4,
  MODE_RTK_FLOAT = 5,
  MODE_RTK_INTEGER = 6,
  MODE_WAAS = 7,
  MODE_OMNISTAR_VBS = 8,
  MODE_OMNISTAR_HP = 9,
  MODE_NO_DATA = 10,
  MODE_BLANKED = 11,
  MODE_DOPLER_PP = 12,
  MODE_SPS_PP = 13,
  MODE_DIFFERENTIAL_PP = 14,
  MODE_RTK_FLOAT_PP = 15,
  MODE_RTK_INTEGER_PP = 16,
  MODE_OMNISTAR_XP = 17,
  MODE_CDGPS = 18,
  MODE_NOT_RECOGNISED = 19,
  MODE_UNKNOWN = 20,
};







































#endif // _DISPATCH_H
