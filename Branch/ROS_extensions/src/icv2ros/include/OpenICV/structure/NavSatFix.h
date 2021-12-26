#ifndef NavSatFix_h 
#define NavSatFix_h 


#include <vector> 
#include <map> 
#include "OpenICV/structure/openicv_ros.h" 
#include "OpenICV/structure/Header.h" 
#include "OpenICV/structure/NavSatStatus.h" 
namespace icv
{
    namespace data
    {
		struct NavSatFix:public PythonDataBase
		{ 
				Header header;
				NavSatStatus status;
				float64 latitude;
				float64 longitude;
				float64 altitude;
				float64 position_covariance[9];
				uint8 COVARIANCE_TYPE_UNKNOWN;
				uint8 COVARIANCE_TYPE_APPROXIMATED;
				uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN;
				uint8 COVARIANCE_TYPE_KNOWN;
				uint8 position_covariance_type;


				enum TAG{  
					COVARIANCE_TYPE_UNKNOWN = 0,
 				 	COVARIANCE_TYPE_APPROXIMATED = 1,
  					COVARIANCE_TYPE_DIAGONAL_KNOWN = 2,
  					COVARIANCE_TYPE_KNOWN = 3
					  };
				MSGPACK_DEFINE(header,status,latitude,longitude,altitude,position_covariance,position_covariance_type);
		
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

	
			memcpy(&(status.status),&message[start_Bit],1);
			start_Bit+=1;
			memcpy(&(status.service),&message[start_Bit],2);
			start_Bit+=2;	


			memcpy(&(latitude),&message[start_Bit],8);
			start_Bit+=8;
			memcpy(&(longitude),&message[start_Bit],8);
			start_Bit+=8;
			memcpy(&(altitude),&message[start_Bit],8);
			start_Bit+=8;
			memcpy(position_covariance,&message[start_Bit],72);
			start_Bit+=72;	
			memcpy(&(position_covariance_type),&message[start_Bit],1);
			start_Bit+=1;
			return  start_Bit;
			
					
		}

				int encode_into_python(std::string message)
		{



			int size=16+header.frame_id.length()+100;
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


			memcpy(buffer+start_Bit, &(status.status),1);
			start_Bit+=1;
			memcpy(buffer+start_Bit, &(status.service),2);
			start_Bit+=2;

			memcpy(buffer+start_Bit, &(latitude),8);
			start_Bit+=8;
			memcpy(buffer+start_Bit, &(longitude),8);
			start_Bit+=8;
			memcpy(buffer+start_Bit, &(altitude),8);
			start_Bit+=8;
			memcpy(buffer+start_Bit,position_covariance,72);
			start_Bit+=72;

			memcpy(buffer+start_Bit, &(position_covariance_type),1);
			start_Bit+=1;
			message.clear();
			message.insert(0,buffer,start_Bit);
			return  start_Bit;




		}		
		
		
		
		
		
		
		
		
		
		
		
		
		}; //struct NavSatFix


	}
}
#endif