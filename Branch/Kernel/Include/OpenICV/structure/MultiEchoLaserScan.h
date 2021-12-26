#ifndef MultiEchoLaserScan_h 
#define MultiEchoLaserScan_h 


#include <vector> 
#include <map> 
#include "OpenICV/structure/openicv_ros.h" 
#include "OpenICV/structure/Header.h"
#include "OpenICV/structure/LaserEcho.h" 

namespace icv
{
    namespace data
    {
struct MultiEchoLaserScan:public PythonDataBase
{ 
		Header header;
		float angle_min;
		float angle_max;
		float angle_increment;
		float time_increment;
		float scan_time;
		float range_min;
		float range_max;
		vector<LaserEcho> ranges;
		vector<LaserEcho> intensities;
		MSGPACK_DEFINE(header,angle_min,angle_max,angle_increment,time_increment,scan_time,range_min,range_max,ranges,intensities);
}; //struct MultiEchoLaserScan
	}}
#endif