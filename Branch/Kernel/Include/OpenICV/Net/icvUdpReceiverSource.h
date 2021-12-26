#ifndef icvUdpReceiverSource_h
#define icvUdpReceiverSource_h

#include "OpenICV/Core/icvFunction.h"
#include <boost/asio.hpp>

#include <boost/asio/streambuf.hpp>
#include <boost/asio/ip/udp.hpp>
namespace icv { namespace function
{
    using namespace icv::core;

    // TODO: support v6
    class icvUdpReceiverSource : public icvFunction
    {
    public:
        icvUdpReceiverSource(icv_shared_ptr<const icvMetaData> info);
        icvUdpReceiverSource();


        virtual void Execute() ICV_OVERRIDE;
        void send(void* buffer_send,int length);

    protected:
        virtual void Process(std::istream& stream) = 0;

    protected:
        boost::asio::ip::udp::endpoint _local, _remote, _send_point;
        boost::asio::streambuf _buffer;
        boost::asio::io_service _service;
        boost::asio::ip::udp::socket _socket;
    };
}}

#endif // icvUdpReceiverSource_hxx
