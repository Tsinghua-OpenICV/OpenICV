//  @ Project : RADAR
//  @ File Name : sensorlrr.h
//  @ Date : 2016/12/22
//  @ Author :
//
//


#ifndef _TuanDynamics_H
#define _TuanDynamics_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
#include "OpenICV/Net/icvUdpReceiverSource.h"


#include <cstdlib>
#include <string>
#include <sstream>
#include"CanFrame4FF.hpp"
#include"CanFrame6FF.hpp"
#include"CanFrame7FF.hpp"
#include"CanFrame43A.hpp"
#include"CanFrame63A.hpp" 
#include"CanFrame73A.hpp" 
#include"CanFrame439.hpp" 
#include"CanFrame639.hpp"
#include"CanFrame731.hpp"
#include"CanFrame738.hpp"
#include"CanFrame739.hpp"
#include"CanFrame766.hpp" 
#include"CanFrame767.hpp" 
#include"CanFrame768.hpp"
#include"CanFrame769.hpp" 
//#include "CanFrame.h"
#include <iomanip>
#include <iostream>
#include <boost/thread/thread.hpp>
#include<cmath>
#include<algorithm>
#include<iostream>  
#include<bitset> 
//#include "boost_udp.h"

typedef unsigned char uint8;
typedef float float32;
typedef double float64;
typedef char int8;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned int uint32;
typedef int int32;
#define PI 3.1415926

using namespace icv;
using namespace icv::function;

class icvMobileye: public icvUdpReceiverSource
{
public:
  icvMobileye(icv_shared_ptr<const icvMetaData> info) : icvUdpReceiverSource(info){
	    };



  virtual void Process(std::istream& stream) override
  {
	  if (_buffer.size() % 13 == 0)
	  {
		  while (!stream.eof())
		  {
			  char udpBuffer[13];
			  stream.read(udpBuffer, 13);

			  int i, j;

			  //lrr
			  int flag = 0;
			  //frames.clear();
			  TimestampedCanFrame Tframe;

			  Tframe.frame.id = ((uint16)(udpBuffer[3] << 8) + udpBuffer[4]);


			  // fill up udpBufferGroup
			  for (j = 0; j < 8; j++)
			  {
				  Tframe.frame.data[j] = udpBuffer[j + 5];
			  }
			  switch (Tframe.frame.id)
			  {
			  case 0x4FF:
				  {testcan4FF.SetData(Tframe);
				  testcan4FF.decode();
				  StructMobileyeObstaclesHeader data1 = *(StructMobileyeObstaclesHeader*)(testcan4FF.data());
				  ICV_LOG_DEBUG <<"4FF: obstacleCount"<< data1.obstacleCount;
				  break;}
			case 0x439: // 0x439+i*3 Obstacle Data A
			case 0x43C:
			case 0x43F:
			case 0x442:
			case 0x445:
			case 0x448:
			case 0x44B:
			case 0x44E:
			case 0x451:
			case 0x454: 
				{
			ob1 = (Tframe.frame.id-0x439)/3;
			testcan439.SetData(Tframe);
			testcan439.decode();
			StructMobileyeObstacle1 data7 = *(StructMobileyeObstacle1*)(testcan439.data());
			ICV_LOG_DEBUG <<"439: id  "<< data7.id<<" ;position:"<<data7.x<<","<<data7.y<<"  ;type:"<<data7.type<<"  ;status:"<<data7.status;
			break;
			}
			case 0x43A: // 0x43A+i*3 Obstacle Data B
			case 0x43D:
			case 0x440:
			case 0x443:
			case 0x446:
			case 0x449:
			case 0x44C:
			case 0x44F:
			case 0x452:
			case 0x455: 
				  {
			ob1 = (Tframe.frame.id-0x43A)/3;
			testcan43A.SetData(Tframe);
			testcan43A.decode();
			StructMobileyeObstacle2 data4 = *(StructMobileyeObstacle2*)(testcan43A.data());
				  ICV_LOG_DEBUG <<"43A: width"<< data4.width<<"age:"<<data4.age;
			break;

			}
			  case 0x6FF:
				 { testcan6FF.SetData(Tframe);
				  testcan6FF.decode();
				  StructMobileyeObstaclesHeader data2 = *(StructMobileyeObstaclesHeader*)(testcan6FF.data());
				  ICV_LOG_DEBUG <<"6FF: obstacleCount"<< data2.obstacleCount;
				  break;}

			case 0x639: // 0x639+i*3 Obstacle Data A
			case 0x63C:
			case 0x63F:
			case 0x642:
			case 0x645:
			case 0x648:
			case 0x64B:
			case 0x64E:
			case 0x651:
			case 0x654: 
				  {
			ob2= (Tframe.frame.id-0x639)/3;	  
			testcan639.SetData(Tframe);
			testcan639.decode();
			StructMobileyeObstacle1 data8 = *(StructMobileyeObstacle1*)(testcan639.data());
			ICV_LOG_DEBUG <<"639: id  "<< data8.id<<" ;position:"<<data8.x<<","<<data8.y<<"  ;type:"<<data8.type<<"  ;status:"<<data8.status;
			break;

			}
			case 0x63A: // 0x63A+i*3 Obstacle Data B
			case 0x63D:
			case 0x640:
			case 0x643:
			case 0x646:
			case 0x649:
			case 0x64C:
			case 0x64F:
			case 0x652:
			case 0x655:
			{
				ob2= (Tframe.frame.id-0x63A)/3;
				testcan63A.SetData(Tframe);
				testcan63A.decode();
			StructMobileyeObstacle2 data5 = *(StructMobileyeObstacle2*)(testcan63A.data());
				 ICV_LOG_DEBUG <<"63A: width"<< data5.width<<"age:"<<data5.age;

			break;
			}
			  case 0x7FF:
				{ testcan7FF.SetData(Tframe);
				  testcan7FF.decode();

				  StructMobileyeObstaclesHeader data3 = *(StructMobileyeObstaclesHeader*)(testcan7FF.data());
				  ICV_LOG_DEBUG <<"7FF: obstacleCount"<< data3.obstacleCount;break;
				  
				  
				  }
			  

			case 0x739: // 0x739+i*3 Obstacle Data A
			case 0x73C:
			case 0x73F:
			case 0x742:
			case 0x745:
			case 0x748:
			case 0x74B:
			case 0x74E:
			case 0x751:
			case 0x754:
				{
			ob3= (Tframe.frame.id-0x739)/3;
			testcan739.SetData(Tframe);
			testcan739.decode();
			StructMobileyeObstacle1 data9 = *(StructMobileyeObstacle1*)(testcan739.data());
			ICV_LOG_DEBUG <<"739: id  "<< data9.id<<" ;position:"<<data9.x<<","<<data9.y<<"  ;type:"<<data9.type<<"  ;status:"<<data9.status;
			break;
			}


			case 0x73A: // 0x73A+i*3 Obstacle Data B
			case 0x73D:
			case 0x740:
			case 0x743:
			case 0x746:
			case 0x749:
			case 0x74C:
			case 0x74F:
			case 0x752:
			case 0x755:
			{
				ob3= (Tframe.frame.id-0x73A)/3;
			testcan73A.SetData(Tframe);
			testcan73A.decode();
			StructMobileyeObstacle2 data6 = *(StructMobileyeObstacle2*)(testcan73A.data());
			ICV_LOG_DEBUG <<"73A: width"<< data6.width<<"age:"<<data6.age;
			break;
			}



			case 0x731: 
			{
				testcan731.SetData(Tframe);
				testcan731.decode();
			StructMobileyeLane data10 = *(StructMobileyeLane*)(testcan731.data());
			ICV_LOG_DEBUG <<"731: lanevalid  "<< data10.LaneValid<<" ;LaneCurvature:"<<data10.LaneCurvature<<" ;LaneHeading:"<<data10.LaneHeading<<"  ;offset:"<<data10.LaneOffset<<"  ;confidence:"<<data10.LaneConf;
			break;
			}
				case 0x738: 
			{
				testcan738.SetData(Tframe);
			testcan738.decode();
			StructMobileyeObstaclesHeader data11 = *(StructMobileyeObstaclesHeader*)(testcan738.data());
			ICV_LOG_DEBUG <<"738: obstacleCount  "<<data11.obstacleCount;
			break;			
			}
		
				  	case 0x766: 
				  {
					  
			testcan766.SetData(Tframe);
			testcan766.decode();
			LaneInfoPart1 data12 = *(LaneInfoPart1*)(testcan766.data());
		    ICV_LOG_DEBUG <<"766: left laneType"<< data12.LaneType<<";  position:"<<data12.LanePositionC0;
			break;

			}
			case 0x767: 
			{
				testcan767.SetData(Tframe);
				testcan767.decode();
			LaneInfoPart2 data13 = *(LaneInfoPart2*)(testcan767.data());
		    ICV_LOG_DEBUG <<"767: left lane heading"<< data13.LaneHeadingC1;
			break;
			}
				case 0x768: 
			{
				testcan768.SetData(Tframe);
			testcan768.decode();
			LaneInfoPart1 data14 = *(LaneInfoPart1*)(testcan768.data());
			 ICV_LOG_DEBUG <<"768: right laneType"<< data14.LaneType<<";  position:"<<data14.LanePositionC0;
			break;			 
			}
				case 0x769: 
				{
			testcan769.SetData(Tframe);
			testcan769.decode();
			LaneInfoPart2 data15 = *(LaneInfoPart2*)(testcan769.data());
		    ICV_LOG_DEBUG <<"769: right lane heading"<< data15.LaneHeadingC1;
			break;
			}
		
		
			  //frames.push_back(Tframe);
		  }


		  // outData[0]->As<icv::opencv::icvCvMatData>() = mFrame;
		//  frames.clear();
	  }//while
	  }//if
	  
	  else
	  {
		  printf("Error: have not received can data, or byte number is wrong!\n");
	  }
	  //outData[0]->As<icv::opencv::icvCvMatData>() = mFrame;
  
  }

  
	
	
private:

  
  static const int MaxUdpBufferSize = 1024;
 // uint8 udpBuffer[MaxUdpBufferSize]; //received udp buffer
 // uint8 sendBuffer[104]; //received udp buffer
CanFrame CanFrame_sent;
  //vector<TimestampedCanFrame> frames;
  CanFrame4FF testcan4FF;
  CanFrame6FF testcan6FF;
  CanFrame7FF testcan7FF;
  CanFrame43A testcan43A;
  CanFrame63A testcan63A;
  CanFrame73A testcan73A;
  CanFrame439 testcan439;
  CanFrame639 testcan639;
  CanFrame739 testcan739;
  CanFrame731 testcan731;
  CanFrame738 testcan738;
  CanFrame766 testcan766;
  CanFrame767 testcan767;
  CanFrame768 testcan768;
  CanFrame769 testcan769;
 int ob1,ob2,ob3;
	std::bitset<64> can_sent; 
	std::bitset<8> temp_8; int startbit,length;
		std::bitset<8> can_sent_bit[8];
  //CanFrame Can_tosent;

   
};
ICV_REGISTER_FUNCTION(icvMobileye)

#endif  //
