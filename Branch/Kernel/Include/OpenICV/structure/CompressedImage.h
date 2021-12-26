#ifndef CompressedImage_h 
#define CompressedImage_h 


#include <vector> 
#include <map> 
#include "OpenICV/structure/openicv_ros.h"  
#include "OpenICV/structure/Header.h"
#include "OpenICV/Core/icvDataObject.h"

namespace icv
{
    namespace data
    {
struct CompressedImage:public PythonDataBase
{ 
		Header header;
		string format;
		vector<uint8> data;
		MSGPACK_DEFINE(header,format,data);
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




			memcpy(&(length_S2),&message[start_Bit],4);
			start_Bit+=4;		
			char format_c[length_S2];
			memcpy(format_c,&message[start_Bit],length_S2);
			format.clear();
			format.insert(0,format_c);
			

			memcpy(&(length_S3),&message[start_Bit],4);
			start_Bit+=4;
			data.resize(length_S3);
			memcpy(&(data[0]),&message[start_Bit],length_S3);
			start_Bit+=length_S3;
			return  start_Bit;
		}


				int encode_into_python(std::string message)
		{

			int size=16+header.frame_id.length()+4+format.length()+4+data.size();			
			//int;
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


			length_S2=format.length();
			memcpy(buffer+start_Bit,&(length_S2),4);
			start_Bit+=4;
			strcpy(buffer+start_Bit, format.c_str());
			start_Bit+=length_S2;


			
			length_S3=data.size();
			memcpy(buffer+start_Bit,&(length_S3),4);
			start_Bit+=4;
			memcpy(buffer+start_Bit,&(data[0]),length_S3);
			start_Bit+=length_S3;
			message.clear();
			message.insert(0,buffer,start_Bit);
			return  start_Bit;

			
		}
}; //struct CompressedImage
	}}
#endif