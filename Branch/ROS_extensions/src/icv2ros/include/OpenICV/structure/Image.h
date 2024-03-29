#ifndef Image_h 
#define Image_h 


#include <vector> 
#include <map> 
#include "OpenICV/structure/openicv_ros.h" 
#include "OpenICV/structure/Header.h"
#include "OpenICV/Core/icvDataObject.h"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp> 

namespace icv
{
    namespace data
    {
struct Image:public PythonDataBase
{ 
		Header header;
		uint32 height;
		uint32 width;
		string encoding;
		uint8 is_bigendian;
		uint32 step;
		vector<uint8> data;
		MSGPACK_DEFINE(header,height,width,encoding,is_bigendian,step,data);

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


			memcpy(&(height),&message[start_Bit],4);
			start_Bit+=4;
			memcpy(&(width),&message[start_Bit],4);
			start_Bit+=4;

			memcpy(&(length_S2),&message[start_Bit],4);
			start_Bit+=4;		
			char endcoding_c[length_S2];
			memcpy(endcoding_c,&message[start_Bit],length_S2);
			encoding.clear();
			encoding.insert(0,endcoding_c);
			start_Bit+=length_S2;
			memcpy(&is_bigendian,&message[start_Bit],1);
			start_Bit+=1;
			memcpy(&step,&message[start_Bit],4);
			start_Bit+=4;

			memcpy(&(length_S3),&message[start_Bit],4);
			start_Bit+=4;
			data.resize(length_S3);
			memcpy(&(data[0]),&message[start_Bit],length_S3);
			start_Bit+=length_S3;
			return  start_Bit;
		}


				int encode_into_python(std::string message)
		{

			int size=16+header.frame_id.length()+12+encoding.length()+9+data.size();			
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

			memcpy(buffer+start_Bit, &(height),4);
			start_Bit+=4;
			memcpy(buffer+start_Bit, &(width),4);
			start_Bit+=4;

			length_S2=encoding.length();
			memcpy(buffer+start_Bit,&(length_S2),4);
			start_Bit+=4;
			strcpy(buffer+start_Bit, encoding.c_str());
			start_Bit+=length_S2;

			memcpy(buffer+start_Bit, &(is_bigendian),1);
			start_Bit+=1;
			memcpy(buffer+start_Bit, &(step),4);
			start_Bit+=4;
			
			length_S3=data.size();
			memcpy(buffer+start_Bit,&(length_S3),4);
			start_Bit+=4;
			memcpy(buffer+start_Bit,&(data[0]),length_S3);
			start_Bit+=length_S3;
			message.clear();
			message.insert(0,buffer,start_Bit);
			return  start_Bit;

			
		}
		//    Image & operator = (const cv::Mat& datum)
        // {
		// 	Image a;
		// 	a.header=datum.header;
		// 	a.height=datum.height;
		// 	a.width=datum.width;
		// 	a.encoding=datum.encoding;
		// 	a.is_bigendian=datum.is_bigendian;
		// 	a.data=datum.data;
        //     return a;
        // }
}; //struct Image
	}
}
#endif