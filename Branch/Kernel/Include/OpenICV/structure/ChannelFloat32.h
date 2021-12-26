#ifndef ChannelFloat32_h 
#define ChannelFloat32_h 


#include <vector> 
#include <map> 
#include "OpenICV/structure/openicv_ros.h"  
namespace icv
{
    namespace data
    {
struct ChannelFloat32:public PythonDataBase
{ 
		string name;
		vector<float> values;
		MSGPACK_DEFINE(name,values);
		int decode_from_python(std::string message)
		{
			
						int length_S,length_S2;int start_Bit;
		
			memcpy(&(length_S),&message[0],4);
			start_Bit+=4;
			char frame_id_c[length_S];
			memcpy(frame_id_c,&message[start_Bit],length_S);
			name.clear();
			name.insert(0,frame_id_c);	
			start_Bit+=length_S;
			memcpy(&(length_S2),&message[start_Bit],4);
			start_Bit+=4;
			values.resize(length_S2);
			memcpy(&(values[0]),&message[start_Bit],length_S2*4);
			start_Bit+=length_S2*4;

						return  start_Bit;





		}
			
			
		int encode_into_python(std::string message)
		{
			int size=8+name.length()+values.size()*4;
				char buffer[size];int start_Bit=0;
				int length_S,length_S2;
				length_S=name.length();
				length_S2=values.size();
				memcpy(buffer+start_Bit,&(length_S),4);
				start_Bit=4;
				strcpy(buffer+start_Bit, name.c_str());
				start_Bit+=length_S;

				memcpy(buffer+start_Bit,&(length_S2),4);
				start_Bit=4;
				memcpy(buffer+start_Bit,&(values[0]),length_S2*4);
				start_Bit+=length_S2*4;

				message.clear();
				message.insert(0,buffer,start_Bit);
				return  start_Bit;
					//	ICV_LOG_INFO<<"TOTAL LENGTH :"<<start_Bit+1;


		}
}; //struct ChannelFloat32
	}}
#endif