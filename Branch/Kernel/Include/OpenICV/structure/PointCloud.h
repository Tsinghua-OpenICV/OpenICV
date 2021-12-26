#ifndef PointCloud_h 
#define PointCloud_h 


#include <vector> 
#include <map> 
#include "OpenICV/structure/openicv_ros.h" 
#include "OpenICV/structure/Header.h"
#include "OpenICV/structure/Point32.h" 
#include "OpenICV/structure/ChannelFloat32.h"
namespace icv
{
    namespace data
    {
struct PointCloud:public PythonDataBase
{ 
		Header header;
		vector<Point32> points;
		vector<ChannelFloat32> channels;
		MSGPACK_DEFINE(header,points,channels);
		int decode_from_python(std::string message)
		{
			int length_S,length_S2,length_S3,size1,size2;int start_Bit;
		
			length_S=header.decode_from_python(message);
			start_Bit+=length_S;



			memcpy(&(length_S2),&message[start_Bit],4);
			start_Bit+=4;
			points.resize(length_S2);
			for(int i=0;i<length_S2;i++)
			{
				start_Bit+=points.at(i).decode_from_python(message.substr(start_Bit));

			}
			memcpy(&(length_S3),&message[start_Bit],4);
			start_Bit+=4;
			channels.resize(length_S3);
			for(int i=0;i<length_S3;i++)
			{
				start_Bit+=channels.at(i).decode_from_python(message.substr(start_Bit));

			}

			
		return start_Bit;


		}

				int encode_into_python(std::string message)
		{
			int length_S,length_S2,length_S3,size1,size2;int start_Bit;
			vector<string> *temp_buffer;vector<int> * temp_count;
			length_S2=points.size();length_S3=channels.size();

			temp_buffer->resize(1+length_S2+length_S3);
			temp_count->resize(1+length_S2+length_S3);

			for(int i=0;i<length_S2+length_S3+1;i++)
			{
				if(i==0)temp_count->at(i)=header.encode_into_python(temp_buffer->at(i));
				else if(i<length_S2+1) temp_count->at(i)=points.at(i-1).encode_into_python(temp_buffer->at(i));
				else temp_count->at(i)=channels.at(i-1-length_S2).encode_into_python(temp_buffer->at(i));
				message+=temp_buffer->at(i);
				start_Bit+=temp_count->at(i);
			}


			


		return start_Bit;


		}



}; //struct PointCloud
	}}
#endif