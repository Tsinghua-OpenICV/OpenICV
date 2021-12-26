#ifndef Imu_h 
#define Imu_h 


#include <vector> 
#include <map> 
#include "OpenICV/structure/openicv_ros.h" 
#include "OpenICV/structure/Header.h" 
#include "OpenICV/structure/Vector3.h" 
#include "OpenICV/structure/Quaternion.h" 

namespace icv
{
    namespace data
    {
struct Imu:public PythonDataBase
{ 
		Header header;
		Quaternion orientation;
		float64 orientation_covariance[9];
		Vector3 angular_velocity;
		float64 angular_velocity_covariance[9];
		Vector3 linear_acceleration;
		float64 linear_acceleration_covariance[9];
		MSGPACK_DEFINE(header,orientation,orientation_covariance,angular_velocity,angular_velocity_covariance,linear_acceleration,linear_acceleration_covariance);
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

			memcpy(&(orientation.x),&message[start_Bit],8);
			start_Bit+=8;
			memcpy(&(orientation.y),&message[start_Bit],8);
			start_Bit+=8;
			memcpy(&(orientation.z),&message[start_Bit],8);
			start_Bit+=8;
			memcpy(&(orientation.w),&message[start_Bit],8);
			start_Bit+=8;

			memcpy(orientation_covariance,&message[start_Bit],72);
			start_Bit+=72;	

			memcpy(&(angular_velocity.x),&message[start_Bit],8);
			start_Bit+=8;
			memcpy(&(angular_velocity.y),&message[start_Bit],8);
			start_Bit+=8;
			memcpy(&(angular_velocity.z),&message[start_Bit],8);
			start_Bit+=8;

			memcpy( angular_velocity_covariance,&message[start_Bit],72);
			start_Bit+=72;

			memcpy(&(linear_acceleration.x),&message[start_Bit],8);
			start_Bit+=8;
			memcpy(&(linear_acceleration.y),&message[start_Bit],8);
			start_Bit+=8;
			memcpy(&(linear_acceleration.z),&message[start_Bit],8);
			start_Bit+=8;

			memcpy(linear_acceleration_covariance,&message[start_Bit],72);
			start_Bit+=72;
			return  start_Bit;			

		}

				int encode_into_python(std::string message)
		{
			int size=16+header.frame_id.length()+296;
			char buffer[size];
			int length_S,length_S2,length_S3;int start_Bit;
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

			memcpy(buffer+start_Bit, &(orientation.x),8);
			start_Bit+=8;
			memcpy(buffer+start_Bit, &(orientation.y),8);
			start_Bit+=8;
			memcpy(buffer+start_Bit, &(orientation.z),8);
			start_Bit+=8;
			memcpy(buffer+start_Bit, &(orientation.w),8);
			start_Bit+=8;
			memcpy(buffer+start_Bit,orientation_covariance,72);
			start_Bit+=72;

			memcpy(buffer+start_Bit, &(angular_velocity.x),8);
			start_Bit+=8;
			memcpy(buffer+start_Bit, &(angular_velocity.y),8);
			start_Bit+=8;
			memcpy(buffer+start_Bit, &(angular_velocity.z),8);
			start_Bit+=8;
			memcpy(buffer+start_Bit,angular_velocity_covariance,72);
			start_Bit+=72;

			memcpy(buffer+start_Bit, &(linear_acceleration.x),8);
			start_Bit+=8;
			memcpy(buffer+start_Bit, &(linear_acceleration.y),8);
			start_Bit+=8;
			memcpy(buffer+start_Bit, &(linear_acceleration.z),8);
			start_Bit+=8;
			memcpy(buffer+start_Bit,linear_acceleration_covariance,72);
			start_Bit+=72;

			message.clear();
			message.insert(0,buffer,start_Bit);
			return  start_Bit;

			
		}

}; //struct Imu
	}}
#endif