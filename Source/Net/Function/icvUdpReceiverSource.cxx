#include "OpenICV/Net/icvUdpReceiverSource.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeOutput.h"
#include "OpenICV/Core/icvNodeInput.h"
using namespace std;
using namespace boost::asio::ip;
int counttt =0;

namespace icv { namespace function
{
    ICV_CONSTEXPR const char KEY_SENDER_ADDRESS[] = "remote_address";
    ICV_CONSTEXPR const char KEY_SENDER_PORT[] = "remote_port";
    ICV_CONSTEXPR const char KEY_RECEIVER_ADDRESS[] = "local_address";
    ICV_CONSTEXPR const char KEY_RECEIVER_PORT[] = "local_port";

    icvUdpReceiverSource::icvUdpReceiverSource() : icvUdpReceiverSource(ICV_NULLPTR) {}
    icvUdpReceiverSource::icvUdpReceiverSource(icv_shared_ptr<const icvMetaData> info)
        : icvFunction(info), _context(), _socket(_context)
    {
        address addr_remote, addr_local;
        unsigned short port_remote, port_local;

        // Construct remote address
         if (_information.Contains(KEY_SENDER_ADDRESS))
         addr_remote = address::from_string(_information.GetString(KEY_SENDER_ADDRESS));
         if (_information.Contains(KEY_SENDER_PORT))
           { port_remote = _information.GetInteger(KEY_SENDER_PORT);
             _remote = udp::endpoint(addr_remote, port_remote);
             } 

        // Construct local address
        if (!_information.Contains(KEY_RECEIVER_PORT))
            ICV_THROW_MESSAGE("Receiver port should be specified!");
        port_local = _information.GetInteger(KEY_RECEIVER_PORT);
        if (_information.Contains(KEY_RECEIVER_ADDRESS))
        {
            addr_local = address::from_string(_information.GetString(KEY_RECEIVER_ADDRESS));
            _local = udp::endpoint(addr_local, port_local);
        }
        
       else _local = udp::endpoint(udp::v4(), port_local);

        _socket.open(_local.protocol());
        _socket.bind(_local);
    }
    void icvUdpReceiverSource::send(void* buffer_send,int length)
    {
         _socket.send_to(boost::asio::buffer(buffer_send,length),_remote);

    }
    void icvUdpReceiverSource::Execute(std::vector<icvNodeInput*>& inputPorts, std::vector<icvNodeOutput*>& outputPorts)
    {
       // ICV_LOG_INFO<<"CONNECTING remote "<<_remote.address().to_string()<<"  port: "<<_remote.port();
        // ICV_LOG_INFO<<"CONNECTING local "<<_local.address().to_string()<<"  port: "<<_local.port();

	    //char receive_buffer[32] = {0};
        IndexType count = _socket.receive_from(_buffer.prepare(1024*8), _send_point);
       // IndexType count =_socket.receive_from(boost::asio::buffer(receive_buffer, 32), _remote);

      //  ICV_LOG_INFO<<"Connected. send buffer size:"<<count;

        _buffer.commit(count);
       
        std::istream dstream(&_buffer);
        Process(dstream);


        _buffer.consume(count);
        counttt++;
        //  ICV_LOG_INFO<<"debug 0--"<<counttt;
    }
}}
