#ifndef PointCloud2_h 
#define PointCloud2_h 


#include <vector> 
#include <map> 
#include "OpenICV/structure/openicv_ros.h"  
#include "OpenICV/structure/Header.h"
#include "OpenICV/structure/PointField.h"
namespace icv
{
    namespace data
    {
struct PointCloud2:public PythonDataBase
{ 
		Header header;
		uint32 height;
		uint32 width;
		vector<PointField> fields;
		bool is_bigendian;
		uint32 point_step;
		uint32 row_step;
		vector<uint8> data;
		bool is_dense;
		MSGPACK_DEFINE(header,height,width,fields,is_bigendian,point_step,row_step,data,is_dense);

		int decode_from_python(std::string message)
		{
			int start_Bit=0;
			//counter->push_back(decode_string_python (name,message));
			start_Bit+=header.decode_from_python(message.substr(start_Bit));
			start_Bit+=decode_T_python (height,message.substr(start_Bit));
			start_Bit+=decode_T_python (width,message.substr(start_Bit));
			int lengthS;
			start_Bit+=decode_T_python (lengthS,message.substr(start_Bit));
			fields.resize(lengthS);
			for(int i=0;i<lengthS;i++)
			start_Bit+=fields.at(i).decode_from_python(message.substr(start_Bit));
			start_Bit+=decode_T_python (is_bigendian,message.substr(start_Bit));
			start_Bit+=decode_T_python (point_step,message.substr(start_Bit));
			start_Bit+=decode_T_python (row_step,message.substr(start_Bit));
			int lengthS2;
			start_Bit+=decode_T_python (lengthS2,message.substr(start_Bit));
			data.resize(lengthS2);
			memcpy(&(data[0]),&message[start_Bit],lengthS2);
			start_Bit+=lengthS2;
			start_Bit+=decode_T_python (is_dense,message.substr(start_Bit));
			

	

			
		return start_Bit;


		}

		int encode_into_python(std::string message)
		{

			int start_Bit=0;
			int size=10+fields.size();
			vector<string> *temp_buffer;vector<int> * temp_count;

			temp_buffer->resize(size);
			temp_count->resize(size);

			message.clear();
			start_Bit+=header.encode_into_python(temp_buffer->at(0));
			message+=temp_buffer->at(0);
			start_Bit+=encode_T_python(height,temp_buffer->at(1));
			message+=temp_buffer->at(1);			
			start_Bit+=encode_T_python(width,temp_buffer->at(2));
			message+=temp_buffer->at(2);
			int Lengths=fields.size();
			start_Bit+=encode_T_python(Lengths,temp_buffer->at(3));
			message+=temp_buffer->at(3);
			for(int i=4;i<4+fields.size();i++)
			{
				start_Bit+=fields.at(i-4).encode_into_python(temp_buffer->at(i));
				message+=temp_buffer->at(i);
			}	
			start_Bit+=encode_T_python(is_bigendian,temp_buffer->at(4+Lengths));
			message+=temp_buffer->at(4+Lengths);
			start_Bit+=encode_T_python(point_step,temp_buffer->at(5+Lengths));
			message+=temp_buffer->at(5+Lengths);
			start_Bit+=encode_T_python(row_step,temp_buffer->at(6+Lengths));
			message+=temp_buffer->at(6+Lengths);

			int length_S3=data.size();
			start_Bit+=encode_T_python(length_S3,temp_buffer->at(7+Lengths));
			message+=temp_buffer->at(7+Lengths);

			char data_tm[length_S3];
			memcpy(data_tm,&(data[0]),length_S3);
			temp_buffer->at(8+Lengths).clear();
			temp_buffer->at(8+Lengths).insert(0,data_tm);	
			start_Bit+=length_S3;

			start_Bit+=encode_T_python(is_dense,temp_buffer->at(9+Lengths));
			message+=temp_buffer->at(9+Lengths);


				
				return  start_Bit;


		}	
}; //struct PointCloud2
	}}
#endif