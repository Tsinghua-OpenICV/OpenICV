#ifndef LaserScan_h 
#define LaserScan_h 


#include <vector> 
#include <map> 
#include "OpenICV/structure/openicv_ros.h" 
#include "OpenICV/structure/Header.h"
namespace icv
{
    namespace data
    {

struct LaserScan:public PythonDataBase
{ 
		Header header;
		float angle_min;
		float angle_max;
		float angle_increment;
		float time_increment;
		float scan_time;
		float range_min;
		float range_max;
		vector<float> ranges;
		vector<float> intensities;
		MSGPACK_DEFINE(header,angle_min,angle_max,angle_increment,time_increment,scan_time,range_min,range_max,ranges,intensities);
    		int decode_from_python(std::string message)
		{
			int length_S,length_S2,length_S3;int start_Bit;
			memcpy(&(header.seq),&message[0],4);
			start_Bit=4;
			memcpy(&(header.stamp.secs),&message[start_Bit],4);
			start_Bit+=4;
			memcpy(&(header.stamp.nsecs),&message[start_Bit],4);
			start_Bit+=4;
			memcpy(&(length_S),&message[start_Bit],4);
			start_Bit+=4;
			char frame_id_c[length_S];
			memcpy(frame_id_c,&message[start_Bit],length_S);
			header.frame_id.clear();
			header.frame_id.insert(0,frame_id_c);
			start_Bit+=length_S;

			memcpy(&(angle_min),&message[start_Bit],4);
			start_Bit+=4;
			memcpy(&(angle_max),&message[start_Bit],4);
			start_Bit+=4;
			memcpy(&(angle_increment),&message[start_Bit],4);
			start_Bit+=4;				
			memcpy(&(time_increment),&message[start_Bit],4);
			start_Bit+=4;
			memcpy(&(scan_time),&message[start_Bit],4);
			start_Bit+=4;
			memcpy(&(range_min),&message[start_Bit],4);
			start_Bit+=4;
			memcpy(&(range_max),&message[start_Bit],4);
			start_Bit+=4;	
			memcpy(&(length_S2),&message[start_Bit],4);
			start_Bit+=4;			
			ranges.resize(length_S2);
			memcpy(&(ranges[0]),&message[start_Bit],length_S2*4);
			start_Bit+=4*length_S2;
			memcpy(&(length_S3),&message[start_Bit],4);
			start_Bit+=4;			
			intensities.resize(length_S3);
			memcpy(&(intensities[0]),&message[start_Bit],length_S3*4);
			start_Bit+=4*length_S3;
			return  start_Bit;

		}
			
			
		int encode_into_python(std::string message)
		{
			int size =16+header.frame_id.length()+28+8+(ranges.size()+intensities.size())*4;
			char buffer[size];
			int length_S=header.frame_id.length();
			int length_S2=ranges.size();
			int length_S3=intensities.size();
			
			
			int start_Bit=0;
			
			memcpy(buffer,&(header.seq),4);
			start_Bit=4;
			memcpy(buffer+start_Bit,&(header.stamp.secs),4);
			start_Bit+=4;
			memcpy(buffer+start_Bit,&(header.stamp.nsecs),4);
			start_Bit+=4;
			length_S=header.frame_id.length();
			memcpy(buffer+start_Bit,&(length_S),4);
			start_Bit+=4;
			strcpy(buffer+start_Bit, header.frame_id.c_str());
			start_Bit+=length_S;

			memcpy(buffer+start_Bit, &(angle_min),4);
			start_Bit+=4;
			memcpy(buffer+start_Bit, &(angle_max),4);
			start_Bit+=4;
			memcpy(buffer+start_Bit, &(angle_increment),4);
			start_Bit+=4;
			memcpy(buffer+start_Bit, &(time_increment),4);
			start_Bit+=4;
			memcpy(buffer+start_Bit, &(scan_time),4);
			start_Bit+=4;
			memcpy(buffer+start_Bit, &(range_min),4);
			start_Bit+=4;
			memcpy(buffer+start_Bit, &(range_max),4);
			start_Bit+=4;



			memcpy(buffer+start_Bit,&(length_S2),4);
			start_Bit+=4;
			memcpy(buffer+start_Bit,&(ranges[0]),length_S2*4);
			start_Bit+=4+length_S2*4;
			memcpy(buffer+start_Bit,&(length_S3),4);
			start_Bit+=4;
			memcpy(buffer+start_Bit,&(intensities[0]),length_S3*4);
			start_Bit+=4+length_S3*4;				
			message.clear();
			message.insert(0,buffer,start_Bit);
			return  start_Bit;
					//	ICV_LOG_INFO<<"TOTAL LENGTH :"<<start_Bit+1;


		}


}; //struct LaserScan
	}}
#endif