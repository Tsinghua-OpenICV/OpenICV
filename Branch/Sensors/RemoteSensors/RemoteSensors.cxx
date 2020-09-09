//  @ Project : RADAR
//  @ File Name : sensorlrr.h
//  @ Date : 2016/12/22
//  @ Author :
//
//


#ifndef _IMUEthernet_H
#define _IMUEthernet_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Net/icvUdpReceiverSource.h"
#include "uartTranslater.h"

#include <cstdlib>
#include <string>
#include <sstream>

#include <boost/thread/thread.hpp>


#define PI 3.1415926


typedef unsigned char uint8;
typedef float float32;
typedef double float64;
typedef char int8;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned int uint32;
typedef int int32;



using namespace icv;
using namespace icv::function;

#include <eigen3/Eigen/Dense>
#include <string>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <unistd.h>
// Packet structure
#include "OpenICV/structure/structureIMU.h"
#include "time.h" 
#include "OpenICV/structure/structure_gps.h"
#include "OpenICV/Basis/icvStructureData.hxx"

// Ethernet

// UINT16_MAX is not defined by default in Ubuntu Saucy



class RemoteSensors: public icvFunction
{
public:

  RemoteSensors(icv_shared_ptr<const icvMetaData> info) : icvFunction(info){

  void* context = zmq_ctx_new ();
  
// zmq receiving sockets
  void* commandSocket = zmq_socket (context, ZMQ_SUB);  
//  zmq_connect (commandSocket, "tcp://127.0.0.1:6970");
//  zmq_connect (commandSocket, "tcp://192.168.1.31:6970");
//  zmq_connect (commandSocket, "tcp://192.168.1.217:6970");
  zmq_connect (commandSocket, "tcp://127.0.0.1:6970");
  zmq_setsockopt (commandSocket, ZMQ_SUBSCRIBE, "", 0);
  std::vector<void*> receivingSocketList;
  receivingSocketList.push_back(commandSocket);

// zmq sending sockets
  void* broadcastingSocket = zmq_socket (context, ZMQ_PUB);
  zmq_bind(broadcastingSocket, "tcp://*:6974");  
  std::vector<void*> sendingSocketList;
  sendingSocketList.push_back(broadcastingSocket);

  std::cout << "zmq initialized." << std::endl;

// thread initial
  boost::asio::io_service io;
  Jobs jobs(io, receivingSocketList, sendingSocketList);
  boost::thread t(boost::bind(&boost::asio::io_service::run, &io));
  io.run();
  t.join();

	    };



      virtual void Execute() override
  {

// Send_Out(msg_imu_t,0);
// Send_Out(msg_fix_t,1);
// Send_Out(msg_vel_t,2);
// Send_Out(msg_odom_t,3);

  }


	
	
private:
    NavSatFix msg_fix;
    TwistWithCovarianceStamped msg_vel;
    Imu msg_imu;
    Odometry msg_odom;
    char pointer_imu[72];

   


};
ICV_REGISTER_FUNCTION(RemoteSensors)

#endif  //
