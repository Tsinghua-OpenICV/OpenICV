
#ifndef _UDP2LOCAL_H
#define _UDP2LOCAL_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Core/icvTime.h"
#include "OpenICV/Core/icvConfig.h"
#include "OpenICV/Basis/icvSemaphore.h"

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
#define REVEIVE_BUFFER 131072

using namespace icv;
using namespace core;
using namespace std;

typedef data::icvStructureData<Imu> icvImu;
typedef data::icvStructureData<TrackArray> icvfusion;

class UDP2LOCAL : public icvFunction
{
public:
    UDP2LOCAL(icv_shared_ptr<const icvMetaData> info) : icvFunction(info) 
	{    
		sockfd = socket(PF_INET, SOCK_DGRAM, 0);
		bzero(&servaddr, sizeof(servaddr));
		servaddr.sin_family = AF_INET;
		servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
		servaddr.sin_port = htons(6981);    
		bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr));

		FD_ZERO(&MasterSet);
    	FD_SET(sockfd, &MasterSet);

		msg_imu = new icvImu() ;
        msg_fusion = new icvfusion() ;
        timesync_=new icvSemaphore(1);
        icv_thread loop(bind(&UDP2LOCAL::InnerLoop, this));	


        count = 0;
			
	}

	virtual ~UDP2LOCAL()
	{
		close(sockfd);
	}


    virtual void Execute(std::vector<icvNodeInput*>& inputPorts, std::vector<icvNodeOutput*>& outputPorts) override
    {
        // timesync_->Lock();   

		Timeout.tv_sec = 3;
        Timeout.tv_usec = 0;

        Res = select(sockfd+1, &MasterSet, NULL, NULL, &Timeout);
        std::cout<<"received: "<<Res<<std::endl;
        if (Res <= 0)
        {
            // CONTINUEL;
        }
        else if(FD_ISSET(sockfd,&MasterSet))
        {
            int Len = REVEIVE_BUFFER;        
            char *recvline=(char *)calloc(1,Len*sizeof(char));
            recvfrom(sockfd, recvline, Len, 0, NULL, NULL);
            nowT = SyncClock::now_us().time_since_epoch().count();
            count++ ;

            int dataType = (int)recvline[LEN_SIZE]-48 ;
            std::cout<<"data type: "<<dataType<<std::endl;
            switch(dataType)
            {
                case 1 : //imu
                {
                    // icvImu imuMsg;
                    // udpMsgDecode(imuMsg,recvline, sourceT);

                    // std::cout<<"time delay: "<<nowT-sourceT<<" us"<<std::endl;
                    
                    // Send_Out(imuSensorMsg,0);
                }
                case 2 : //fusion
                {
                    icvfusion fusionMsg ;
                    udpMsgDecode(fusionMsg, recvline, sourceT);
                    std::cout<<"time delay: "<<nowT-sourceT<<" us"<<std::endl;
                }
            }
            udpMsgDelete(recvline);   
        }

        CONTINUEL:
        FD_ZERO(&MasterSet);
        FD_SET(sockfd, &MasterSet);
		
    }
private:
    void InnerLoop()
    {
        while(true) 
        {
            timesync_->Release();
            usleep(100000);
        }
    }
    icvSemaphore* timesync_;

	int sockfd;
    struct sockaddr_in servaddr;
    int Res = 0;
    struct timeval Timeout;
    fd_set MasterSet;
	icvImu *msg_imu ;
    icvfusion *msg_fusion ;
    int count ;
    time_t nowT, sourceT;
};

ICV_REGISTER_FUNCTION(UDP2LOCAL)

#endif 