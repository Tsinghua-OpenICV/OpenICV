#ifndef __ICVMSGENCODE_H__
#define __ICVMSGENCODE_H__

#include "OpenICV/structure/structureIMU.h"
#include "OpenICV/structure/structureFusionData.h"
#include "OpenICV/structure/RadarRecordReplay.h"
#include <stdio.h>
#include <unistd.h>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include <msgpack.hpp>



#define WSMP_HDR_SIZE 32 //计算机到DSRC之间的消息头长度
#define MSG_BUFFER 800
#define LEN_SIZE 8 //消息的最大长度 10^n

using namespace msgpack;
using namespace std ;

template <class sensorMsgT>
char*
udpMsgEncode(const sensorMsgT &icvMsg, int &Len, time_t sourceTime, std::string type)
{
    // pack input data
    std::stringstream buff ;
    pack(buff, icvMsg);
    // add data info
    std::stringstream out ;
    size_t buffLen = buff.str().size();
    out <<sourceTime<<" "<<buffLen<<" "<<buff.rdbuf();

    char *udpMsg = NULL ;
    out.seekg(0,std::ios::end);
    int length = out.tellg();
    out.seekg(0,std::ios::beg);
    Len = length + LEN_SIZE + 1;
    udpMsg = new char[Len];
    memset(udpMsg, 0, Len);
    ICV_LOG_INFO<<"data length: "<<length<<" buffer size: "<<Len;

    // msg string length
    std::ostringstream strLen;
    strLen << std::setw(LEN_SIZE) << std::setfill('0')<<Len ;
    strcpy(udpMsg, strLen.str().c_str());
    strcpy(udpMsg+LEN_SIZE, type.c_str());

    //add data into udp msg 
    out.read(udpMsg+LEN_SIZE+1,length);

    return udpMsg;

}




template <class sensorMsgT>
void
udpMsgDecode(sensorMsgT &icvMsg, char *udpMsg, time_t &sourceTime)
{
    // get input msg length
    int inLen = 0 ;
    
    for (int i = 0; i < LEN_SIZE; i++)
    {
        inLen = inLen + (int)(pow(10, (LEN_SIZE-1-i))*((int)udpMsg[i]-48));
    }  
    
    // convert char* to ss
    std::stringstream in ;
    in.write(udpMsg+LEN_SIZE+1, inLen-LEN_SIZE-1);

    //unpack msg
    unpacked result;
    string buff ;
    ostringstream outstring;
    size_t len ;

    in >>  sourceTime >> len ;
    in >> outstring.rdbuf();
    buff = outstring.str();

    ICV_LOG_INFO<<"buff length: "<<inLen<<" sourceTime: "<<sourceTime<<" data length: "<<len ;

    unpack(&result, buff.c_str(), len);
    object obj = result.get();
    TrackArray msg ;
    // Imu msg ;
    obj.convert(msg);
    icvMsg.setoutvalue(msg, sourceTime);
}

void
udpMsgDelete(char* udpMsg);


inline void udpMsgDelete(char* udpMsg)
{
    free(udpMsg);
}

#endif


