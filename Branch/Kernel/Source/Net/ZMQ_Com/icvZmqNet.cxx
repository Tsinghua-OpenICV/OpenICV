
#ifndef _ZMQNET_H
#define _ZMQNET_H


#include "OpenICV/Net/icvUdpReceiverSource.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvTime.h"
#include <zmq.hpp>

#include "OpenICV/Core/icvConfig.h"
#include <boost/asio/streambuf.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio.hpp>
using namespace std;
using namespace boost::asio::ip;
using namespace icv;
using namespace icv::function;


class icvZmqNet : public icvFunction
{
    public:
    icvZmqNet() : icvZmqNet(ICV_NULLPTR) {}
    icvZmqNet(icv_shared_ptr<const icvMetaData> info): icvFunction(info)
    {

            if (_information.Contains("buffer_size"))
         buffer_size_=_information.GetInteger("buffer_size");
         else buffer_size_=2048;

		 void* context = zmq_ctx_new ();
  
		// // zmq receiving sockets
		//   void* receivesocket = zmq_socket (context, ZMQ_SUB);  
		// //  zmq_connect (commandSocket, "tcp://127.0.0.1:6970");
		// //  zmq_connect (commandSocket, "tcp://192.168.1.31:6970");
		// //  zmq_connect (commandSocket, "tcp://192.168.1.217:6970");
		//   zmq_bind (receivesocket, "tcp://127.0.0.1:6970");
		//   zmq_setsockopt (receivesocket, ZMQ_SUBSCRIBE, "", 0);
		//   receivingSocketList.push_back(receivesocket);

		// zmq sending sockets

        string sub1="socket1";
        Register_Sub(sub1);
        void* receivesocket = zmq_socket (context, ZMQ_SUB);  
        zmq_connect (receivesocket, "tcp://127.0.0.1:6970");
        zmq_setsockopt (receivesocket, ZMQ_SUBSCRIBE, "", 0);
        receivingSocket_withnames.emplace(sub1,receivesocket);


        string pub1="socket2";
        Register_Pub(pub1);
        void* broadcastingSocket = zmq_socket (context, ZMQ_PUB);
        zmq_bind(broadcastingSocket, "tcp://*:6974");  
        sendingSocket_withnames.emplace(sub1,broadcastingSocket);

        std::cout << "zmq initialized." << std::endl;
			
		
    }
    void send(void* buffer_send,int length,string name)
    {
        // _socket.send_to(boost::asio::buffer(buffer_send,length),_remote);
		  zmq_send(sendingSocket_withnames[name], buffer_send,length, 0);
	   
	   


    }

       int receive(void* buffer_receive,int length,string name)
    {

        zmq_pollitem_t items[1];
        items[0].socket = receivingSocket_withnames.at(name);
        items[0].events = ZMQ_POLLIN;
        zmq_poll(items, 1, 0);
        int size_;
        if (items[0].revents& ZMQ_POLLIN)
             size_= zmq_recv(items[0].socket, buffer_receive, length, 0);
        return size_;
    

    }
    void Process(std::istream &zmq_socket_data)
    {


    }
    void Execute()
    {
       // char msg[buffer_ize_] = {0};
    
       

    //std::cout << "Polling Information ..." << std::endl;
        char mybuffer_[buffer_size_] = {0};
        int size=receive(mybuffer_,buffer_size_,get_sub_names()->at(0));
        if (size != -1)
        {
        std::stringbuf _buffer;
        _buffer.sputn(mybuffer_,size);
		//_buffer.commit(count);
        std::istream dstream(&_buffer);
        Process(dstream);

        }
		
	   


    }

    protected:
   // vector<void *> receivingSocketList;
    icv_map<string,void *> receivingSocket_withnames;
    icv_map<string,void *> sendingSocket_withnames;

   // vector<void *> sendingSocketList;
    int buffer_size_=2048;

};
ICV_REGISTER_FUNCTION(icvZmqNet)



#endif
