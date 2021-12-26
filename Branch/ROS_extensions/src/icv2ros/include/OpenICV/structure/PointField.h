#ifndef PointField_h 
#define PointField_h 


#include <vector> 
#include <map> 
#include "OpenICV/structure/openicv_ros.h"  

namespace icv
{
    namespace data
    {
struct PointField:public PythonDataBase
{ 
enum Format{
	  INT8 = 1,
  UINT8 = 2,
  INT16 = 3,
  UINT16 = 4,
  INT32 = 5,
  UINT32 = 6,
  FLOAT32 = 7,
  FLOAT64 = 8
};
		string name;
		uint32 offset;
		uint8 datatype;
		uint32 count;
		MSGPACK_DEFINE(name,offset,datatype,count);
		int decode_from_python(std::string message)
		{
			//vector<int> *counter; 
			int start_Bit=0;
			//counter->push_back(decode_string_python (name,message));
			start_Bit+=decode_string_python (name,message.substr(start_Bit));
			start_Bit+=decode_T_python (offset,message.substr(start_Bit));
			start_Bit+=decode_T_python (datatype,message.substr(start_Bit));
			start_Bit+=decode_T_python (count,message.substr(start_Bit));

			return  start_Bit;
		}
			
			
		int encode_into_python(std::string message)
		{

			int start_Bit=0;
			vector<string> *temp_buffer;vector<int> * temp_count;

			temp_buffer->resize(4);
			temp_count->resize(4);

			message.clear();
			start_Bit+=encode_string_python(name,temp_buffer->at(0));
			message+=temp_buffer->at(0);
			start_Bit+=encode_T_python(offset,temp_buffer->at(1));
			message+=temp_buffer->at(1);			
			start_Bit+=encode_T_python(datatype,temp_buffer->at(2));
			message+=temp_buffer->at(2);
			start_Bit+=encode_T_python(count,temp_buffer->at(3));
			message+=temp_buffer->at(3);	
				
				return  start_Bit;


		}		


}; //struct PointField
	}}
#endif