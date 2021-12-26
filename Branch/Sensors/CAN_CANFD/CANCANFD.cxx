
#ifndef _CANCANFD_H
#define _CANCANFD_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Core/icvTime.h"
#include "OpenICV/Core/icvConfig.h"


#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/thread/thread.hpp>
#include <iostream>

#include <string>
#include <sstream>
#include <vector>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <time.h>
#include <sys/time.h>


using namespace icv;
using namespace core;
using namespace std;

class CANCANFD : public icvFunction
{
public:
   
    CANCANFD(icv_shared_ptr<const icvMetaData> info) : icvFunction(info) {
	 ICV_LOG_INFO << "CAN Tool Started";
       if (_information.Contains("mode")){
         can_mode = _information.GetString("mode");
	   }
	   else{
         can_mode = "can";
	   }
        if (_information.Contains("status")){
         status = _information.GetString("status");
	   }
	   else{
         status = "-r";
	   }       
   if (_information.Contains("device_name")){
         device_name = _information.GetString("device_name");
	   }
	   else{
         device_name = "can0";
	   }   
    if (can_mode=="can"){
      if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
	{
	    perror("Create socket failed");
	    exit(-1);
	}
    /* set up can interface */
	strcpy(ifr.ifr_name, device_name.data());
	printf("can port is %s\n",ifr.ifr_name);
	/* assign can device */
	ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
    /* bind can device */
	if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		perror("Bind can device failed\n");
		close(s);
		exit(-2);
	}

    	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
	{
	    perror("Create socket failed");
	    exit(-1);
	}
       if(status=="-r"){
		       /* configure receiving */
	    /* set filter for only receiving packet with can id 0x1F */
		rfilter[0].can_id = 0x1F;
	    rfilter[0].can_mask = CAN_SFF_MASK;
	    if(setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0)
	    {
	    	perror("set receiving filter error\n");
	    	close(s);
	    	exit(-3);
	    }
	    /* keep reading */
		tv.tv_sec=0;
		tv.tv_usec = 20000;
	   }
	   else if(status=="-s"){
		printf("sending\n");
		tv.tv_sec=0;
		tv.tv_usec=50000;
	   }
	}
	else if (can_mode=="canfd"){
	enable_canfd = 1;
	/* create socket */
	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
	{
	    perror("Create socket failed");
	    exit(-1);
	}
/* set up can interface */
	strcpy(ifr.ifr_name, device_name.data());
	printf("can port is %s\n",ifr.ifr_name);
	/* assign can device */
	ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
    /* bind can device */
	if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		perror("Bind can device failed\n");
		close(s);
		exit(-2);
	}
	
	if(setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0)
	{
		perror("set receiving filter error\n");
		close(s);
		exit(-3);
	}
	}
	



	}

    ~CANCANFD(){ ICV_LOG_INFO << "CAN Tool Finished";	}

    virtual void Execute() override
    {
		if (can_mode=="can"){
             if(status=="-r"){
           
	    /* keep reading */
	
			select(0, NULL,NULL,NULL, &tv);
	        nbytes = read(s, &frame, sizeof(frame));
	        if(nbytes > 0)
	        {
	        	printf("%s ID=%#x data length=%d\n", ifr.ifr_name, frame.can_id, frame.can_dlc);
	        	for (int i=0; i < frame.can_dlc; i++)
	        		printf("%#x ", frame.data[i]);
	        	printf("\n");
	        }
	

			 } 
			 if(status=="-s"){
        	select(s+1, NULL,NULL,NULL, &tv);
			/* configure can_id and can data length */
			frame.can_id = 0x1F;
			frame.can_dlc = 8;
			printf("%s ID=%#x data length=%d\n", ifr.ifr_name, frame.can_id, frame.can_dlc);
			/* prepare data for sending: 0x11,0x22...0x88 */
			for (int i=0; i<8; i++)
			{
				frame.data[i] = ((i+1)<<4) | (i+1);
				printf("%#x ", frame.data[i]);
			}
			printf("Sent out\n");
			/* Sending data */
			if(write(s, &frame, sizeof(frame)) < 0)
			{
				perror("Send failed");
				close(s);
				exit(-4);
			}
			 } 
		}
		else if (can_mode=="canfd"){
            if(status=="-r"){
				nbytes = recv(s, &frame_fd, CANFD_MTU, 0 );
				//nbytes = read(s, &frame_fd, CANFD_MTU);
				if(nbytes > 0)
				{
					if (nbytes == CANFD_MTU)
					{
						printf("got CAN FD frame with length %d, flags = %d\n",  frame_fd.len,  frame_fd.flags);
					}
					else if (nbytes == CAN_MTU) 
					{
						printf("got legacy CAN frame with length %d, flags = %d\n", frame_fd.len, frame_fd.flags);
					}
					printf("%s ID=%#x \n", ifr.ifr_name, frame_fd.can_id);
					for (int i=0; i < frame_fd.len; i++)
						printf("%#x ", frame_fd.data[i]);
					printf("\n");
				}
				else
				{
					printf("read can error\n");
				}
			 } 
			 if(status=="-s"){
                	printf("sending\n");
		struct timeval tv;

		/* configure can_id and can data length */
		frame_fd.can_id = 0x1F;
		frame_fd.len = 62;
		printf("%s ID=%#x data length=%d\n", ifr.ifr_name, frame_fd.can_id, frame_fd.len);
		/* prepare data for sending: 0x11,0x22...0x88 */
		for (int i=0; i< frame_fd.len; i++)
		{
			frame_fd.data[i] = i;
		}
		printf("Sent out\n");
		/* Sending data */
		if(write(s, &frame_fd, CANFD_MTU) < 0)
		{
			perror("Send failed");
			close(s);
			exit(-4);
		}
			 } 
		}
    }
private:
string can_mode, status, device_name;
int s, nbytes;

sockaddr_can addr;
ifreq ifr;
can_frame frame;
can_filter rfilter[1];
timeval tv;
int enable_canfd = 0;

canfd_frame frame_fd;


};

ICV_REGISTER_FUNCTION(CANCANFD)


#endif  //
