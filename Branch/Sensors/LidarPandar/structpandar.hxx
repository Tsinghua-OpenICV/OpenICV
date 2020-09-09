#ifndef __STRUCTPANDAR_H
#define __STRUCTPANDAR_H



#include <errno.h>
#include <stdint.h>
#include <string>
#include <boost/format.hpp>
#include <math.h>

#include "pcl/point_cloud.h"


static uint16_t DATA_PORT_NUMBER = 8080;     // default data port
  static uint16_t POSITION_PORT_NUMBER = 8308; // default position port

  class PandarPacket
  {
public:
    PandarPacket(){data.resize(1240);

    };
    ~PandarPacket(){};
int32_t stamp;
std::vector<uint8_t> data;
  };
typedef struct {
    std::string calibrationFile;     ///< calibration file name
    double max_range;                ///< maximum range to publish
    double min_range;                ///< minimum range to publish
    int min_angle;                   ///< minimum angle to publish
    int max_angle;                   ///< maximum angle to publish
    double view_direction;
    double view_width;
    double tmp_min_angle;
    double tmp_max_angle;
    double start_angle;
    std::string frame_id;            ///< tf frame ID
    std::string model;               ///< device model name
    int    npackets;                 ///< number of packets to collect
    double rpm;                      ///< device rotation rate (RPMs)
    double repeat_delay;
    double time_offset;              ///< time in seconds added to each pandar time stamp
    std::string ip_add;
    std::string pcapfile;
    int ip_port;
    bool read_fast;
    bool read_once;
    }Config_pandar;
struct PandarGps
{
int32_t stamp;
uint32_t used;
uint16_t year;
uint16_t month;
uint16_t day;
uint16_t hour;
uint16_t minute;
uint16_t second;
uint32_t fineTime;
uint32_t flag;

};

  

struct Header

{

uint32_t seq;
int32_t stamp;
std::string frame_id;

};

  class PandarScan
  {
public:
    PandarScan(){


    };
    ~PandarScan(){};
Header header;
std::vector<PandarPacket> packets;
  };



  typedef boost::shared_ptr<PandarScan> PandarScanPtr;
  typedef boost::shared_ptr< PandarGps> PandarGpsPtr;

  typedef boost::shared_ptr<const PandarScan> PandarScanConstPtr;
  typedef boost::shared_ptr<const PandarGps> PandarGpsConstPtr;

      static inline double from_degrees(double degrees)
    {
      return degrees * M_PI / 180.0;
    }
   
    static inline double to_degrees(double radians)
    {
       return radians * 180.0 / M_PI;
    }















#endif