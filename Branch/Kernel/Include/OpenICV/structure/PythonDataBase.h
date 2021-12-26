#ifndef PythonDataBase_hxx
#define PythonDataBase_hxx

#include <string>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include "OpenICV/Core/icvDataObject.h"
using namespace std;
namespace icv 
{ 
	namespace data
	{
		struct PythonDataBase
		{
			public:
			PythonDataBase(){} ;
			~PythonDataBase(){} ;

			virtual int decode_from_python(std::string message){};
			virtual int encode_into_python(std::string message){};
			template<typename T> int decode_T_python(T data,std::string message)
		{
			memcpy(&(data),&message[0],sizeof(T));
			return  sizeof(T);
		}
			
			
		template<typename T> int encode_T_python(T data,std::string &message)
		{
				char buffer[sizeof(T)];
				int start_Bit=0;
				memcpy(buffer,&(data),sizeof(T));
				start_Bit+=sizeof(T);				
				message.clear();
				message.insert(0,buffer,start_Bit);
				return  start_Bit;
					//	ICV_LOG_INFO<<"TOTAL LENGTH :"<<start_Bit+1;
		}

		 int decode_string_python(string data,std::string message)
		{
			int lengthS,start_Bit=0;
			memcpy(&(lengthS),&message[0],4);
			start_Bit+=4;
			memcpy(&(data),&message[0],lengthS);
			start_Bit+=lengthS;

			return  start_Bit;
		}
			
			
		int encode_string_python(string data,std::string &message)
		
		{
			int lengthS=data.length();
				char buffer[lengthS+4 ];
				int start_Bit=0;
				memcpy(buffer+start_Bit,&(lengthS),4);
				start_Bit+=4;
				strcpy(buffer+start_Bit,data.c_str());
				start_Bit+=lengthS;				
				message.clear();
				message.insert(0,buffer,start_Bit);
				return  start_Bit;
					//	ICV_LOG_INFO<<"TOTAL LENGTH :"<<start_Bit+1;
		}


		};
	}

}

#endif