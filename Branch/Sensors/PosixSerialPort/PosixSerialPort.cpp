

#include "PosixSerialPort.h"
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

using namespace icv;
PosixSerialPort::~PosixSerialPort() {
   
}

PosixSerialPort::PosixSerialPort() {


    
}


//////////////////////////////////////////////////////////////////////////
// return the number of frames that have not been yet decoded
//////////////////////////////////////////////////////////////////////////

int PosixSerialPort::numberOfFrames() {
   
    return dataFrameList.size();
}



//////////////////////////////////////////////////////////////////////////
// return a pointer to the first frame in the list
//////////////////////////////////////////////////////////////////////////

buffFrame * PosixSerialPort::firstFrame() {
    return dataFrameList.front();
}


//////////////////////////////////////////////////////////////////////////
// return a pointer to the first frame in the queue and removes it
// The reader is responsible for deleting the frame.
//////////////////////////////////////////////////////////////////////////
buffFrame * PosixSerialPort::getNextFrame()
{
    buffFrame *frame = NULL;
    frameLock_.lock();
    if (dataFrameList.size()>0) {
        frame = dataFrameList.front();
        dataFrameList.pop_front();
    }
    frameLock_.unlock();
    return frame;
}


//////////////////////////////////////////////////////////////////////////
// remove the first frame of the list
// use this function as soon as you copy the frame
//////////////////////////////////////////////////////////////////////////

int PosixSerialPort::removeFirstFrame() {
    frameLock_.lock();

    buffFrame * tmp = dataFrameList.front();
    if (tmp == NULL) {
        std::printf("Not possible to delete the first item of the dataframe list. It is empty.");
        return 0;
    }
    dataFrameList.pop_front();
    //delete[] tmp->data;
    delete tmp;
    tmp = NULL;

    frameLock_.unlock();

    return 1;
}

/*!< open the port 'name'
return true if success
 */
bool PosixSerialPort::openPort(const char * name) {
    handlePort = open(name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (handlePort == -1) {
        std::printf("openPort : Unable to open serial port %s",name);
        return false;
    } else {
        fcntl(handlePort, F_SETFL, 0);
        printf("Port %s has been sucessfully opened and % d is the file description\n", name, handlePort);
        return true;
    }
}


/*!< close the port
return true if success
 */
int PosixSerialPort::closePort() {
  close(handlePort);
}

/*!< configure the port
return true if success
 */
int PosixSerialPort::configurePort(long baudRate, int byteSize, char parity, int stopBits) {

    struct termios port_settings; // structure to store the port settings in

    tcgetattr(handlePort, &port_settings);

    switch (baudRate) {
        case 4800:
            cfsetispeed(&port_settings, B4800); // set baud rates
            cfsetospeed(&port_settings, B4800);
            break;

        case 9600:
            cfsetispeed(&port_settings, B9600); // set baud rates
            cfsetospeed(&port_settings, B9600);
            break;
        case 38400:
            cfsetispeed(&port_settings, B38400); // set baud rates
            cfsetospeed(&port_settings, B38400);
            break;
        case 115200:
            cfsetispeed(&port_settings, B115200); // set baud rates
            cfsetospeed(&port_settings, B115200);
            break;
        default:
            break;
    }

    port_settings.c_cflag &= ~PARENB; // set no parity, stop bits, data bits
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CSIZE;
    port_settings.c_cflag |= CS8;

    tcsetattr(handlePort, TCSANOW, &port_settings); // apply the settings to the port

}



//////////////////////////////////////////////////////////////////////////
// Read 'maxLength' bytes on the port and copy them in buffer
// return the number of bytes read
//////////////////////////////////////////////////////////////////////////
int PosixSerialPort::readBuffer(char *buffer, int maxLength)
{
  int countread = read( handlePort,buffer,maxLength);
  //printf("read %d of %d, data:%s\n", countread, maxLength, buffer);
  return countread;
}

int PosixSerialPort::writeBuffer(string &data_buf, int maxLength)
{
  int countread = write( handlePort,data_buf.c_str(),maxLength);
  return countread;
}


void PosixSerialPort::run()
{

    numberBytesToRead = nbBytesToRead();
    if (numberBytesToRead > 0)
    {
      t_ = icv::icvTime::time_ns();//icv::icvTime::now().time_since_epoch().count();                     // datation
      receivedBuffer_ = new char[numberBytesToRead];
      memset(receivedBuffer_,0,numberBytesToRead);
      numberBytesRead = readBuffer(receivedBuffer_,numberBytesToRead);
      processIncomingData();
    }
    else
    {
      receivedBuffer_ = NULL;
      // todo : trouver une autre methode plus efficace que le polling et le sleep !
      usleep(10000);
    }
  
}


int PosixSerialPort::nbBytesToRead()
{
    int bytes;

    ioctl(handlePort, FIONREAD, &bytes);

    return bytes;
}

//////////////////////////////////////////////////////////////////////////
/// Process the data received by the processIncomingEvent() function
/// It may be either bytes (data) or a ring indicator signal (PPS for GPS signal)
void PosixSerialPort::processIncomingData()
{
    // data frame
    if (numberBytesRead > 0) {
    buffFrame * frame = new buffFrame;
    frame->time_stamp_ = t_;
    frame->length_ = numberBytesRead;
    frame->data.resize(numberBytesRead) ;
    memcpy( &(frame->data[0]),  receivedBuffer_, numberBytesRead);;
    frameLock_.lock();
    dataFrameList.push_back( frame );
    frameLock_.unlock();
    delete[] receivedBuffer_;
    receivedBuffer_ = NULL;
  }


  t_ = 0;
  numberBytesToRead = 0;
}
