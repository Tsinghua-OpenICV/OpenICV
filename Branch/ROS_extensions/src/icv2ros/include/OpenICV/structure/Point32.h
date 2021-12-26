#ifndef Point32_h 
#define Point32_h 


#include <vector> 
#include <map> 
#include "OpenICV/structure/openicv_ros.h"  

namespace icv
{
    namespace data
    {
struct Point32:public PythonDataBase
{ 
		float x;
		float y;
		float z;
		MSGPACK_DEFINE(x,y,z);
		int decode_from_python(std::string message)
		{
			memcpy(&(x),&message[0],4);
			memcpy(&(y),&message[4],4);
			memcpy(&(z),&message[8],4);
			return  12;
		}
			
			
		int encode_into_python(std::string message)
		{
				char buffer[12];
				int start_Bit=0;
				memcpy(buffer+start_Bit,&(x),4);
				start_Bit+=4;
				memcpy(buffer+start_Bit,&(y),4);
				start_Bit+=4;
				memcpy(buffer+start_Bit,&(z),4);
				start_Bit+=4;
	
				
				message.clear();
				message.insert(0,buffer,start_Bit);
				return  12;
					//	ICV_LOG_INFO<<"TOTAL LENGTH :"<<start_Bit+1;


		}

}; //struct Point32
	}}
#endif