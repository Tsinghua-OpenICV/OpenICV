#ifndef _LOCAL2UDP_H
#define _LOCAL2UDP_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Basis/icvSemaphore.h"
#include "OpenICV/Core/icvTime.h"
#include "OpenICV/Core/icvConfig.h"
#include "OpenICV/structure/icvCvMatData.hxx"
#include "OpenICV/Basis/icvStructureData.hxx"
#include "OpenICV/structure/structureIMU.h"


#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <unistd.h>
#include <sstream>
#include "icvMsgEncode.hpp"
#include <fstream>

#define WSMP_HDR_SIZE 32
#define REVEIVE_BUFFER 1024

using namespace icv;
using namespace core;
using namespace std;

typedef data::icvStructureData<Imu> icvImu;
typedef data::icvStructureData<TrackArray> icvfusion;
typedef data::icvStructureData<RadarLongOut> icvlrr ;
typedef struct transferData
{
    double number[5550];
    int len ;
    double value;
    MSGPACK_DEFINE(number, len, value);
}transferData;

class LOCAL2UDP : public icvFunction
{
public:
    LOCAL2UDP(icv_shared_ptr<const icvMetaData> info) : icvFunction(info) 
	{    	
        sockfd = socket(PF_INET, SOCK_DGRAM, 0);
        //connect to the DSRC devices
        bzero(&servaddr, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(6980);//port
        servaddr.sin_addr.s_addr = inet_addr("192.168.253.10");//ip address
        count = 0;
        timesync_=new icvSemaphore(1);
        icv_thread loop(bind(&LOCAL2UDP::InnerLoop, this));

	}

	virtual ~LOCAL2UDP()
	{
        close(sockfd);
	}


    virtual void Execute(std::vector<icvNodeInput*>& inputPorts, std::vector<icvNodeOutput*>& outputPorts) override
    {
        timesync_->Lock();
        msg_imu = read_Input<icvImu>(0); //imu data
        msg_fusion = read_Input<icvfusion>(1); // fusion data
        // time_t source_time = read_Input<icvImu>(0).GetSourceTime();
        time_t source_time = SyncClock::now_us().time_since_epoch().count();

        int Len ;
        std::string dataType = "2" ; // 1 for imu; 2 for fusion 
        //pass the data to encode function
        char *udpMsg = udpMsgEncode(msg_fusion,Len,source_time,dataType);        
        int flag = sendto(sockfd, udpMsg, Len, 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
        udpMsgDelete(udpMsg);	
    }
private:
    void InnerLoop()
    {
        while(true) 
        {
            timesync_->Release();
            usleep(10000);
        }
    }
    icvSemaphore* timesync_;
    int sockfd;
    struct sockaddr_in servaddr;
    int count ;
    Imu msg_imu ;
    TrackArray msg_fusion ;
    RadarLongOut msg_radar ;
    string str ;
	
};

ICV_REGISTER_FUNCTION(LOCAL2UDP)

#endif 