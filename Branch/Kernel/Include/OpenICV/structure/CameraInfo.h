#ifndef CameraInfo_h 
#define CameraInfo_h 


#include <vector> 
#include <map> 
#include "OpenICV/structure/openicv_ros.h" 
#include "OpenICV/structure/Header.h"
#include "OpenICV/structure/RegionOfInterest.h"
#include "OpenICV/Core/icvDataObject.h"

namespace icv
{
    namespace data
    {
			struct CameraInfo:public PythonDataBase
			{ 
				public:
					Header header;
					uint32 height;
					uint32 width;
					string distortion_model;
					vector<float64> D;
					float64 K[9];
					float64 R[9];
					float64 P[12];
					uint32 binning_x;
					uint32 binning_y;
					RegionOfInterest roi;
					MSGPACK_DEFINE(header,height,width,distortion_model,D,K,R,P,binning_x,binning_y,roi);
			int decode_from_python(std::string &message)
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
						//header.frame_id=std::string(frame_id_c);
					//	ICV_LOG_INFO<<"NAME LENGTH "<<length_S<<"  FRAME ID:"<<header.frame_id;

						start_Bit+=length_S;
						memcpy(&(height),&message[start_Bit],4);
						start_Bit+=4;
						memcpy(&(width),&message[start_Bit],4);
						start_Bit+=4;
						memcpy(&(length_S2),&message[start_Bit],4);
						start_Bit+=4;
						char distortion_model_c[length_S2];
						memcpy(distortion_model_c,&message[start_Bit],length_S2);
						//distortion_model=std::string(distortion_model_c);
						start_Bit+=length_S2;
						distortion_model.clear();
						distortion_model.insert(0,distortion_model_c);
					//ICV_LOG_INFO<<"MODEL LENGTH "<<length_S2<<"  distortion_model:"<<distortion_model;

						memcpy(&(length_S3),&message[start_Bit],4);
						start_Bit+=4;
						D.resize(length_S3);
						memcpy(&(D[0]),&message[start_Bit],length_S3*8);
						start_Bit+=length_S3*8;
					//	ICV_LOG_INFO<<"D LENGTH"<<length_S3;

						memcpy(K,&message[start_Bit],72);
						start_Bit+=72;			
						memcpy(R,&message[start_Bit],72);
						start_Bit+=72;					
					//	ICV_LOG_INFO<<"IN THE CAMERA INFO4";

						memcpy(P,&message[start_Bit],96);
						start_Bit+=96;			
						memcpy(&(binning_x),&message[start_Bit],4);
						start_Bit+=4;
						memcpy(&(binning_y),&message[start_Bit],4);
						start_Bit+=4;
						memcpy(&(roi.x_offset),&message[start_Bit],4);
						start_Bit+=4;
						memcpy(&(roi.y_offset),&message[start_Bit],4);
						start_Bit+=4;
						memcpy(&(roi.height),&message[start_Bit],4);

						start_Bit+=4;
						memcpy(&(roi.width),&message[start_Bit],4);
						start_Bit+=4;
						memcpy(&(roi.do_rectify),&message[start_Bit],1);
						start_Bit+=1;
						return  start_Bit;
					//	ICV_LOG_INFO<<"TOTAL LENGTH :"<<start_Bit+1;


			}
					int encode_into_python(std::string &message)
			{
				std::cout<<"Starting to encode into python"<<endl;
				int size=16+header.frame_id.length()+16+8*D.size()+distortion_model.length()+265;
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
						//header.frame_id=std::string(frame_id_c);
					//	ICV_LOG_INFO<<"NAME LENGTH "<<length_S<<"  FRAME ID:"<<header.frame_id;

						start_Bit+=length_S;

						memcpy(buffer+start_Bit, &(height),4);
						start_Bit+=4;
						memcpy(buffer+start_Bit, &(width),4);
						start_Bit+=4;
						length_S2=distortion_model.length();
						memcpy(buffer+start_Bit, &(length_S2),4);
						start_Bit+=4;
						strcpy(buffer+start_Bit,distortion_model.c_str());
						//ICV_LOG_INFO<<"MODEL LENGTH "<<length_S2<<"  distortion_model:"<<distortion_model;

						start_Bit+=length_S2;
						
						length_S3=D.size();
						memcpy(buffer+start_Bit,&(length_S3),4);
						start_Bit+=4;
						memcpy(buffer+start_Bit,&(D[0]),length_S3*8);
						start_Bit+=length_S3*8;
					//	ICV_LOG_INFO<<"D LENGTH"<<length_S3;

						memcpy(buffer+start_Bit,K,72);
						start_Bit+=72;			
						memcpy(buffer+start_Bit,R,72);
						start_Bit+=72;					
					//	ICV_LOG_INFO<<"IN THE CAMERA INFO4";

						memcpy(buffer+start_Bit,P,96);
						start_Bit+=96;			
						memcpy(buffer+start_Bit,&(binning_x),4);
						start_Bit+=4;
						memcpy(buffer+start_Bit,&(binning_y),4);
						start_Bit+=4;
						memcpy(buffer+start_Bit,&(roi.x_offset),4);
						start_Bit+=4;
						memcpy(buffer+start_Bit,&(roi.y_offset),4);
						start_Bit+=4;
						memcpy(buffer+start_Bit,&(roi.height),4);

						start_Bit+=4;
						memcpy(buffer+start_Bit,&(roi.width),4);
						start_Bit+=4;
						memcpy(buffer+start_Bit,&(roi.do_rectify),1);
						start_Bit+=1;
						message.clear();
						message.insert(0,buffer,start_Bit);
					//	ICV_LOG_INFO<<"TOTAL LENGTH :"<<start_Bit+1;
                        std::cout<<"the message got is "<<message<<endl; 
						return  start_Bit;

			}


			}; //struct CameraInfo
	}
}
#endif