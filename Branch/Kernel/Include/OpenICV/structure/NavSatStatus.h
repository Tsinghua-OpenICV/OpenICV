#ifndef NavSatStatus_h 
#define NavSatStatus_h 


#include <vector> 
#include <map> 
#include "OpenICV/structure/openicv_ros.h" 

namespace icv
{
    namespace data
    {
struct NavSatStatus
{ 
	enum Tag{

	STATUS_NO_FIX = -1,
  STATUS_FIX = 0,
  STATUS_SBAS_FIX = 1,
  STATUS_GBAS_FIX = 2,
  SERVICE_GPS = 1,
  SERVICE_GLONASS = 2,
  SERVICE_COMPASS = 4,
  SERVICE_GALILEO = 8
	};

		int8 status;
		uint16 service;
		MSGPACK_DEFINE(status,service);
    		int decode_from_python(std::string message)
		{
			memcpy(&(status),&message[0],1);
			memcpy(&(service),&message[1],2);
			return  3;

		}
			
			
		int encode_into_python(std::string message)
		{
				char buffer[3];
				memcpy(buffer,&(status),1);
				memcpy(buffer,&(service),2);
				
				message.clear();
				message.insert(0,buffer,3);
				return  3;
					//	ICV_LOG_INFO<<"TOTAL LENGTH :"<<start_Bit+1;


		}

}; //struct NavSatStatus
	}}
#endif