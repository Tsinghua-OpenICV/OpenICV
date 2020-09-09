
#include "OpenICV/Basis/icvZmqNet.h"
#include <boost/asio/streambuf.hpp>



namespace icv{namespace function{

using namespace icv::core;
using namespace std;
ICV_CONSTEXPR const char KEY_SENDER_ADDRESS[] = "pub_address";
ICV_CONSTEXPR const char KEY_RECEIVER_ADDRESS[] = "sub_address";

ZmqNetSource::ZmqNetSource(icv_shared_ptr<const icvMetaData> info)
    : icvFunction(info)
{  
    string addr_remote, addr_local ;
    if (_information.Contains(KEY_SENDER_ADDRESS))
        addr_remote = _information.GetString(KEY_SENDER_ADDRESS);
    if (_information.Contains(KEY_RECEIVER_ADDRESS))
        addr_local = _information.GetString(KEY_RECEIVER_ADDRESS);
    if (!_information.Contains(KEY_SENDER_ADDRESS))
        ICV_THROW_MESSAGE("Publisher port should be specified!");
    if (!_information.Contains(KEY_RECEIVER_ADDRESS))
        ICV_THROW_MESSAGE("Receiver port should be specified!");
    
    char sub_address[256], pub_address[256];
    int i;
    for( i=0;i<addr_local.length();i++)
        sub_address[i] = addr_local[i];
    sub_address[i] = '\0';
    for( i=0;i<addr_remote.length();i++)
        pub_address[i] = addr_remote[i];
    pub_address[i] = '\0';

    context = zmq_ctx_new ();
    // zmq receiving sockets
    subscriber = zmq_socket(context, ZMQ_SUB);
    int rc = zmq_connect (subscriber, sub_address);
    assert (rc == 0);
    zmq_setsockopt (subscriber, ZMQ_SUBSCRIBE, "", 0);

    // zmq sending sockets
    publisher = zmq_socket(context, ZMQ_PUB);
    rc = zmq_bind (publisher, pub_address);
    assert (rc == 0);
    
    ICV_LOG_INFO << "zmq initialized." ;	
}
ZmqNetSource::~ZmqNetSource()
{
    zmq_close(subscriber);
    zmq_close(publisher);
    zmq_ctx_destroy(context);
}


void ZmqNetSource::Execute()
{
    Process();
}
void ZmqNetSource::send(void* buffer_send,int length)
{
    zmq_send(publisher, buffer_send, length, 0);
}
int ZmqNetSource::recv()
{
    int size = zmq_recv(subscriber, _buffer, 2048, 0);
    if (size==-1)
    {
        ICV_LOG_ERROR<<"zmq receive error !";
        return -1;
    }
    else
    {
        return size;
    }
    
}


}}
