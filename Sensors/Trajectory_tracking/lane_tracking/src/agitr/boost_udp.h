#ifndef BOOST_UDP_H
#define BOOST_UDP_H

#include "boost/algorithm/string.hpp"
#include "boost/regex.hpp"
#include "boost/asio.hpp"
#include "boost/thread.hpp"
#include "boost/lexical_cast.hpp"

using namespace boost;
using namespace std;

#define RECVSIZE 1024
class Boost_UDP
{
public:

   Boost_UDP(boost::asio::io_service &io_service, string pcIP, int pcPort, string canetIP, int canetPort):udp_sock(io_service)
   {
       m_canetIP = canetIP;
	   m_canetPort = canetPort;
	   m_pcIP = pcIP;
	   m_pcPort = pcPort;
   }

   ~Boost_UDP()
   {
	   udp_sock.close();
   }

   void start_sock()
   {
	   // here the ip can change to 192.168.1.33
	    boost::asio::ip::udp::endpoint local_add(boost::asio::ip::address_v4::from_string(m_pcIP),m_pcPort);
		udp_sock.open(local_add.protocol());
		udp_sock.bind(local_add);
   }

   int receive_data(unsigned char buf[])
   {
	    //boost::mutex::scoped_lock lock(mutex);
	    //donot change 目的端口与发送端口现在是不一样的
	    boost::asio::ip::udp::endpoint send_endpoint(boost::asio::ip::address_v4::from_string(m_pcIP),m_pcPort);
	    int ret = udp_sock.receive_from(boost::asio::buffer(buf,RECVSIZE),send_endpoint);//堵塞模式
	    return ret;
   }

   int send_data(unsigned char str[], int len)
   {
	   //boost::mutex::scoped_lock lock(mutex);
	   //donot change
	   boost::asio::ip::udp::endpoint send_endpoint(boost::asio::ip::address_v4::from_string(m_canetIP),m_canetPort); 
	   int ret = udp_sock.send_to(boost::asio::buffer(str,len),send_endpoint);

	   return ret;
   }
public:

	string m_canetIP;
	int m_canetPort;
	string m_pcIP;
	int m_pcPort;

   boost::asio::ip::udp::socket udp_sock;
   mutable boost::mutex mutex;
   
};


#endif
