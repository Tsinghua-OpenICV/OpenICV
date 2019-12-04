#include "uartTranslater.h"

int main(int argc, char* argv[])
{  
  std::cout << "program started." << std::endl;

//zmq communication
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

  //printf("Stop to send \n");
  //canetUdpS.closeUdpCanet();
  return 0;
}