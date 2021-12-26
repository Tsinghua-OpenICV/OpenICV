/**
 * @file XUdp.cpp
 * @brief udp package transmission.
 * @author yong.zhu@novauto.com.cn
 * @version 0.0.1
 * @date 2019-10-28
 */

#include <string.h>
#include <stdio.h>
#include <ros/ros.h>
#include "XUdp.h"

int
XUdp::Send (const char *IP, const unsigned short port, const void *buf,
	    int len)
{
  struct sockaddr_in addr;
  memset (&addr, 0, sizeof (addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons (port);	//host to net short
  addr.sin_addr.s_addr = inet_addr (IP);

  //unsigned long laddr = inet_addr("192.168.6.200");
  //unsigned char *p = &laddr;
  //printf("%u, %u, %u, %u\n", *(p), *(p+1), *(p+2), *(p+3));

  size_t rc = 0;

  rc = sendto (st, buf, len, 0, (struct sockaddr *) &addr, sizeof (addr));

  return rc;
}

int
XUdp::Receive (void *buf, int len, char *srcIP)
{
  struct sockaddr_in sendaddr;
  memset (&sendaddr, 0, sizeof (sendaddr));

  socklen_t addrlen;
  addrlen = sizeof (sendaddr);

  memset (buf, 0, len);
  size_t rc = recvfrom (st, buf, len, 0, (struct sockaddr *) &sendaddr, &addrlen);
  if (srcIP)
    strcpy (srcIP, inet_ntoa (sendaddr.sin_addr));
  //printf("%s:\n%s\n", srcIP, buf);
  return rc;
}

XUdp::XUdp ()
{
  st = socket (AF_INET, SOCK_DGRAM, 0);
  if(st == -1){
    ROS_ERROR("canbus - create socket fail");
    return;
  }      
}

XUdp::~XUdp ()
{
  close (st);
}

int
XUdp::Bind (const unsigned short port)
{
  struct sockaddr_in addr;
  memset (&addr, 0, sizeof (addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons (port);
  addr.sin_addr.s_addr = htonl (INADDR_ANY);
  //addr.sin_addr.s_addr = inet_addr("192.168.110.105");

  return bind (st, (struct sockaddr *) &addr, sizeof (addr));
}
