#ifndef TimeReference_h 
#define TimeReference_h 


#include <vector> 
#include <map> 
#include "OpenICV/structure/openicv_ros.h" 
#include "OpenICV/structure/Header.h"
namespace icv
{
    namespace data
    {

struct TimeReference:public PythonDataBase
{ 
		Header header;
		Time time_ref;
		string source;
		MSGPACK_DEFINE(header,time_ref,source);

		int decode_from_python(std::string message)
		{
			//vector<int> *counter; 
			int start_Bit=0;
			//counter->push_back(decode_string_python (name,message));
			start_Bit+=header.decode_from_python(message.substr(start_Bit));
			start_Bit+=time_ref.decode_from_python(message.substr(start_Bit));
			start_Bit+=decode_string_python (source,message.substr(start_Bit));
			return  start_Bit;
		}
			
			
		int encode_into_python(std::string message)
		{

			int start_Bit=0;
			vector<string> *temp_buffer;vector<int> * temp_count;

			temp_buffer->resize(3);
			temp_count->resize(3);

			message.clear();
			start_Bit+=header.encode_into_python(temp_buffer->at(0)); 
			message+=temp_buffer->at(0);
			start_Bit+=time_ref.encode_into_python(temp_buffer->at(1));
			message+=temp_buffer->at(1);			
			start_Bit+=encode_string_python(source,temp_buffer->at(2));
			message+=temp_buffer->at(2);

				
				return  start_Bit;


		}	
}; //struct TimeReference
	}
	}
#endif