#ifndef LaserEcho_h 
#define LaserEcho_h 


#include <vector> 
#include <map> 
#include "OpenICV/structure/openicv_ros.h"  
namespace icv
{
    namespace data
    {

struct LaserEcho:public PythonDataBase
{ 
		vector<float> echoes;
		MSGPACK_DEFINE(echoes);
    		int decode_from_python(std::string message)
		{
			int length_S;int start_Bit=0;
			memcpy(&(length_S),&message[start_Bit],4);
			start_Bit+=4;
			echoes.resize(length_S);
			memcpy(&(echoes[0]),&message[start_Bit],length_S*4);
			start_Bit+=4*length_S;
			return  start_Bit;


		}
			
			
		int encode_into_python(std::string message)
		{
			char buffer[4+4*echoes.size()];
			int length_S=echoes.size();int start_Bit=0;
			memcpy(buffer+start_Bit,&(length_S),4);
			start_Bit+=4;
			memcpy(buffer+start_Bit,&(echoes[0]),length_S*4);
			start_Bit+=4+length_S*4;
				
			message.clear();
			message.insert(0,buffer,start_Bit);
			return  start_Bit;
					//	ICV_LOG_INFO<<"TOTAL LENGTH :"<<start_Bit+1;


		}

}; //struct LaserEcho
	}}
#endif