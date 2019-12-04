#include <boost/test/unit_test.hpp>

#include "OpenICV/Net/icvUdpReceiverSource.h"

#include <thread>

BOOST_AUTO_TEST_SUITE(icvUdpReceiverSource)

using namespace std;
using namespace icv;
using namespace icv::core;
using namespace boost::asio::ip;

int content;
int content_length;

class TestReceiver : public icv::function::icvUdpReceiverSource
{
public:
    TestReceiver(icv_shared_ptr<const icvMetaData> info) : icvUdpReceiverSource(info) {}

    virtual void ConfigurateOutput(std::vector<icvNodeOutput*>&) ICV_OVERRIDE {}
    virtual void Process(std::istream& stream, icvDataObject** outData) ICV_OVERRIDE 
    {
        content_length = _buffer.size();
        stream >> content;
    }
};

BOOST_AUTO_TEST_CASE(execute)
{
    std::string addr = "127.0.0.1", port = "50005";

    icv_shared_ptr<icvMetaData> config(new icvMetaData);
    config->SetString("remote_address", addr);
    config->SetString("local_port", port);

    std::thread trecv([config]
    {
        TestReceiver receiver(config);
        receiver.Execute(ICV_NULLPTR, ICV_NULLPTR); 
    });
    std::this_thread::sleep_for(std::chrono::microseconds(100));

    boost::asio::io_context context;
    udp::socket socket(context, udp::endpoint(udp::v4(), 0));
    char sendcontent[6] = "12345";
    udp::resolver resolver(context);
    udp::resolver::results_type endpoints = resolver.resolve(udp::v4(), "127.0.0.1", port);
    socket.send_to(boost::asio::buffer(sendcontent, 6), *endpoints.begin());

    trecv.join();
    BOOST_CHECK_EQUAL(content, 12345);
    BOOST_CHECK_EQUAL(content_length, 6);
}


BOOST_AUTO_TEST_SUITE_END()
