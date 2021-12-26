#ifndef Point_h 
#define Point_h 


#include <vector> 
#include <map> 
#include "OpenICV/structure/openicv_ros.h"  
namespace icv
{
    namespace data
    {
struct Point:public PythonDataBase
{ 
		float64 x;
		float64 y;
		float64 z;
		MSGPACK_DEFINE(x,y,z);
				int decode_from_python(std::string message)
		{
			memcpy(&(x),&message[0],8);
			memcpy(&(y),&message[8],8);
			memcpy(&(z),&message[16],8);
			return  24;
		}
			
			
		int encode_into_python(std::string message)
		{
				char buffer[24];
				int start_Bit=0;
				memcpy(buffer+start_Bit,&(x),8);
				start_Bit+=8;
				memcpy(buffer+start_Bit,&(y),8);
				start_Bit+=8;
				memcpy(buffer+start_Bit,&(z),8);
				start_Bit+=8;
	
				
				message.clear();
				message.insert(0,buffer,start_Bit);
				return  24;
					//	ICV_LOG_INFO<<"TOTAL LENGTH :"<<start_Bit+1;


		}
}; //struct Point
	}}
#endif