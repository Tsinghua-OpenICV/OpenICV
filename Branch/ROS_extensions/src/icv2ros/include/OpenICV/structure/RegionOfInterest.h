#ifndef RegionOfInterest_h 
#define RegionOfInterest_h 


#include <vector> 
#include <map> 
#include "OpenICV/structure/openicv_ros.h" 

namespace icv
{
    namespace data
    {
struct RegionOfInterest
{ 
		uint32 x_offset;
		uint32 y_offset;
		uint32 height;
		uint32 width;
		bool do_rectify;
		MSGPACK_DEFINE(x_offset,y_offset,height,width,do_rectify);
		int decode_from_python(std::string message)
		{
			memcpy(&(x_offset),&message[0],4);
			memcpy(&(y_offset),&message[4],4);
			memcpy(&(height),&message[8],4);
			memcpy(&(width),&message[12],4);
			memcpy(&(do_rectify),&message[16],1);

		}
			
			
		int encode_into_python(std::string message)
		{
				char buffer[17];
				int start_Bit=0;
				memcpy(buffer+start_Bit,&(x_offset),4);
				start_Bit+=4;
				memcpy(buffer+start_Bit,&(y_offset),4);
				start_Bit+=4;
				memcpy(buffer+start_Bit,&(height),4);
				start_Bit+=4;
				memcpy(buffer+start_Bit,&(width),4);
				start_Bit+=4;
				memcpy(buffer+start_Bit,&(width),1);
				start_Bit+=1;

				message.clear();
				message.insert(0,buffer,start_Bit);
					//	ICV_LOG_INFO<<"TOTAL LENGTH :"<<start_Bit+1;


		}
}; //struct RegionOfInterest
	}}
#endif