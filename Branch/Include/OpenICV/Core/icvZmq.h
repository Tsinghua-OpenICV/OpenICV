#ifndef icvZMQ_h
#define icvZMQ_h

#define ICV_CONFIG_POINTERS
#define ICV_CONFIG_FUNCTION
#include "OpenICV/Core/icvConfig.h"
#undef ICV_CONFIG_POINTERS
#undef ICV_CONFIG_FUNCTION

#include <vector>
#include "OpenICV/Core/icvObject.h"
#include "OpenICV/Core/icvMacros.h"
#include <numeric>
#include <zmq.hpp>
#include <sstream>
#include <boost/asio.hpp>
#include <boost/asio/streambuf.hpp>
namespace icv { namespace core
{
using namespace std;

///////


class ZMQ_Socket 
{
public:
    ZMQ_Socket()
    {

                void*    zmq_soc = NULL;

                void*    context_ = NULL;

                bool    use_uniq_cont=false;

    };
    ZMQ_Socket(void* context_i)
    {

                    zmq_soc = NULL;

                    context_ = context_i;
                    use_uniq_cont=true;


    };

    ~ZMQ_Socket()
    {
                if(zmq_soc) 
                {
                zmq_close(zmq_soc);
                zmq_soc = NULL;
                }
                if(context_) 
                {
                zmq_ctx_destroy(context_);
                context_ = NULL;
                }



    };

    bool socket_ini(int Type, const string &addr, int sendpipe = 1000, int recvpipe = 1000, int timeout = 100)
    {
            if(use_uniq_cont!=true)
            {
            context_ = zmq_ctx_new();
           // ICV_LOG_INFO<<"error of creat CONTEXT "<<zmq_strerror (errno);
           
            zmq_soc = zmq_socket(context_, Type);
          // ICV_LOG_INFO<<"error of creat socket "<<zmq_strerror (errno);

            }
            else
            {

              zmq_soc= zmq_socket(context_, Type);
           // ICV_LOG_INFO<<"error of creat socket "<<zmq_strerror (errno);
           // ICV_LOG_INFO<<"zmq socket ptr "<<zmq_soc;

            //zmq_soc = zmq_socket(context_, Type);

            }
            


            if(!zmq_soc) {ICV_LOG_INFO<<"ZMQ INITIATE FAILED"; return false;}

            switch(Type) 
            {
            case ZMQ_REP:
            case ZMQ_PULL:
            case ZMQ_PUB:
                
                zmq_bind(zmq_soc, addr.c_str());
                ICV_LOG_INFO<<"error of bind socket "<<zmq_strerror (errno);

                break;
            case ZMQ_REQ:
            case ZMQ_PUSH:
            case ZMQ_SUB:
                zmq_connect(zmq_soc, addr.c_str());
               // ICV_LOG_INFO<<"error of connect socket "<<zmq_strerror (errno);

                zmq_setsockopt (zmq_soc, ZMQ_SUBSCRIBE, "", 0);
               // ICV_LOG_INFO<<"error of SET socket "<<zmq_strerror (errno);

                break;
            default:
                return false;
            }
            zmq_setsockopt(zmq_soc, ZMQ_SNDHWM, &sendpipe, sizeof(sendpipe));
            zmq_setsockopt(zmq_soc, ZMQ_RCVHWM, &recvpipe, sizeof(recvpipe));
            zmq_setsockopt(zmq_soc, ZMQ_SNDTIMEO, &timeout, sizeof(timeout));
            adress_send = addr;

            return true;
    };

    bool socket_send(void *buffer, int length)
    {
                zmq_msg_t message;
                zmq_msg_init_size(&message, length);
                memcpy(zmq_msg_data(&message), (char *)buffer, length);
                int count = zmq_msg_send(&message, zmq_soc, 0);

                if(count==length) return true;
                else 	return false;
    };


    bool socket_send(const string &stringdata)
    {
                int length=stringdata.size();
                zmq_msg_t message;
                zmq_msg_init_size(&message, length);
                memcpy(zmq_msg_data(&message), stringdata.c_str(), length);
                int count = zmq_msg_send(&message,zmq_soc, 0);

                //int count = zmq_send(zmq_soc,stringdata.c_str(), length, 0);
                
                //ICV_LOG_INFO<<"error of send "<<zmq_strerror (errno);
                string dat;
                dat.assign((char *)zmq_msg_data(&message), length);

                //ICV_LOG_INFO <<" SEND COUNT "<<count ;
                if(count==length) return true;
                else 	return false;
    };

    bool socket_recv(void *buffer)
    {

                zmq_msg_t message;
                zmq_msg_init(&message);
                int count = zmq_msg_recv(&message, zmq_soc, 0);


                memcpy(buffer, (char *)zmq_msg_data(&message), count);
                if(zmq_msg_close(&message)) return false;

                return true;




    };

    bool socket_recv(string &stringdata)
    {





                zmq_msg_t message;
                zmq_msg_init(&message);
                //ICV_LOG_INFO<<" recv ";
                int count = zmq_msg_recv(&message, zmq_soc, 0);
                //ICV_LOG_INFO<<"RECV COUNT "<<count;
                //ICV_LOG_INFO<<"  erro of recv "<<zmq_strerror(errno) ;
                stringdata.assign((char *)zmq_msg_data(&message), count);
                if(zmq_msg_close(&message)) return false;
		return true; 

    };

    void GetSendAddr(string &addrOut) 
    {
                addrOut = adress_send;
    }

    private:
    void *context_;
    void *zmq_soc;
    string adress_send;  
    bool use_uniq_cont=false;
    };


    }}

    #endif // ICV_ZMQ
