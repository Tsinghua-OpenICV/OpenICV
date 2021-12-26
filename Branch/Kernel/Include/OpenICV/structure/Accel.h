#ifndef Accel_h 
#define Accel_h 


#include <vector> 
#include <map> 
#include "OpenICV/structure/openicv_ros.h" 
#include "OpenICV/structure/Vector3.h"
#include "msgpack.hpp"
#include "OpenICV/Core/icvDataObject.h"

namespace icv
{
    namespace data
    {
struct Accel:public PythonDataBase
{ 
		Vector3 linear;
		Vector3 angular;
		MSGPACK_DEFINE(linear,angular);

		int decode_from_python(std::string message)
		{
			memcpy(&(linear.x),&message[0],8);
			memcpy(&(linear.y),&message[8],8);
			memcpy(&(linear.z),&message[16],8);
			memcpy(&(angular.x),&message[24],8);
			memcpy(&(angular.y),&message[32],8);
			memcpy(&(angular.z),&message[40],8);
			return 48;
		}
			
			
		int encode_into_python(std::string message)
		{
				char buffer[48];
				int start_Bit=0;
				memcpy(buffer+start_Bit,&(linear.x),8);
				start_Bit+=8;
				memcpy(buffer+start_Bit,&(linear.y),8);
				start_Bit+=8;
				memcpy(buffer+start_Bit,&(linear.z),8);
				start_Bit+=8;
				memcpy(buffer+start_Bit,&(angular.x),8);
				start_Bit+=8;
				memcpy(buffer+start_Bit,&(angular.y),8);
				start_Bit+=8;
				memcpy(buffer+start_Bit,&(angular.z),8);
				start_Bit+=8;
				
				message.clear();
				message.insert(0,buffer,start_Bit);
				return start_Bit;

					//	ICV_LOG_INFO<<"TOTAL LENGTH :"<<start_Bit+1;


		}

}; //struct Accel

	}
}
#endif
