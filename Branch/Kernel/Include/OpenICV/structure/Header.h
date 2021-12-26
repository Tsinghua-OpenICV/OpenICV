#ifndef Header_h 
#define Header_h 


#include <vector> 
#include <map> 
#include "OpenICV/structure/openicv_ros.h" 
#include "OpenICV/structure/rosTime.h" 
namespace icv
{
    namespace data
    {
struct Header:public PythonDataBase
{ 
		uint32 seq;
		Time stamp;
		string frame_id;
		MSGPACK_DEFINE(seq,stamp,frame_id);

		int decode_from_python(std::string message)
		{
			int length_S,length_S2,length_S3;int start_Bit;
			memcpy(&(seq),&message[0],4);
			start_Bit=4;
			memcpy(&(stamp.secs),&message[start_Bit],4);
			start_Bit+=4;
			memcpy(&(stamp.nsecs),&message[start_Bit],4);
			start_Bit+=4;
			memcpy(&(length_S),&message[start_Bit],4);
			start_Bit+=4;
			char frame_id_c[length_S];
			memcpy(frame_id_c,&message[start_Bit],length_S);
			frame_id.clear();
			frame_id.insert(0,frame_id_c);	
			start_Bit+=length_S;
			return  start_Bit;
		}		

				int encode_into_python(std::string message)
		{
			int size=16+frame_id.length();
			char buffer[size];
			int length_S,length_S2,length_S3;int start_Bit;
			memcpy(buffer,&(seq),4);
			start_Bit=4;
			memcpy(buffer+start_Bit,&(stamp.secs),4);
			start_Bit+=4;
			memcpy(buffer+start_Bit,&(stamp.nsecs),4);
			start_Bit+=4;
			length_S=frame_id.length();
			memcpy(buffer+start_Bit,&(length_S),4);
			start_Bit+=4;
			strcpy(buffer+start_Bit, frame_id.c_str());
			start_Bit+=length_S;
			message.clear();
			message.insert(0,buffer,start_Bit);
			return  start_Bit;
		}
}; //struct Header
	}}
#endif