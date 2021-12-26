#ifndef Time_h 
#define Time_h 


#include <vector> 
#include <map> 
#include "OpenICV/structure/openicv_ros.h"  
namespace icv
{
    namespace data
    {
struct Time:public PythonDataBase
{ 
		uint secs;
		uint nsecs;
		MSGPACK_DEFINE(secs,nsecs);
		int decode_from_python(std::string message)
		{
			//vector<int> *counter; 
			int start_Bit=0;
			//counter->push_back(decode_string_python (name,message));
			start_Bit+=decode_T_python (secs,message.substr(start_Bit));
			start_Bit+=decode_T_python (nsecs,message.substr(start_Bit));

			return  start_Bit;
		}
			
			
		int encode_into_python(std::string message)
		{

			int start_Bit=0;
			vector<string> *temp_buffer;vector<int> * temp_count;

			temp_buffer->resize(2);
			temp_count->resize(2);

			message.clear();
			start_Bit+=encode_T_python(secs,temp_buffer->at(0));
			message+=temp_buffer->at(0);
			start_Bit+=encode_T_python(nsecs,temp_buffer->at(1));
			message+=temp_buffer->at(1);

				
				return  start_Bit;
		}



}; //struct Time
	}
	}
#endif