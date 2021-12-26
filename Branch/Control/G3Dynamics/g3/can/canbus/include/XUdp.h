#ifndef UDP_H
#define UDP_H

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>

#define SOCKET int

class XUdp
{
private:
  SOCKET st;
public:
  XUdp ();
  ~XUdp ();

  int Send (const char *IP, const unsigned short port, const void *buf,
	    int len);
  int Receive (void *buf, int len, char *srcIP);
  int Bind (const unsigned short port);
};

#endif // UDP_H
