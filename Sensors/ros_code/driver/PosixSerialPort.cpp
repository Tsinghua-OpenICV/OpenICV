/*********************************************************************
// created:    2011/05/18
// filename:   PosixSerialPort.cpp
//
// author:     Gerald Dherbomez
//
// version:    $Id: $
//
// purpose:    This class defines a unix (32bits) driver for serial port
 *********************************************************************/

#include "PosixSerialPort.h"

#include <QDebug>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>


PosixSerialPort::~PosixSerialPort() {
    qDebug() << "The Posix serial port " << componentName << " was destroyed";
}

PosixSerialPort::PosixSerialPort(QString name) {
    componentName = name;
    qDebug() << "The Posix serial port " << componentName << " was created";
}


//////////////////////////////////////////////////////////////////////////
// return the number of frames that have not been yet decoded
//////////////////////////////////////////////////////////////////////////

int PosixSerialPort::numberOfFrames() {
    if (dataFrameList.isEmpty())
        return 0;
    return dataFrameList.count();
}



//////////////////////////////////////////////////////////////////////////
// return a pointer to the first frame in the list
//////////////////////////////////////////////////////////////////////////

FRAME * PosixSerialPort::firstFrame() {
    return dataFrameList.first();
}


//////////////////////////////////////////////////////////////////////////
// return a pointer to the first frame in the queue and removes it
// The reader is responsible for deleting the frame.
//////////////////////////////////////////////////////////////////////////
FRAME * PosixSerialPort::getNextFrame()
{
    FRAME *frame = NULL;
    frameLock_.lock();
    if (!dataFrameList.isEmpty()) {
        frame = dataFrameList.takeFirst();
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

    FRAME * tmp = dataFrameList.first();
    if (tmp == NULL) {
        qWarning("Not possible to delete the first item of the dataframe list. It is empty.");
        return 0;
    }
    dataFrameList.removeFirst();
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
        qWarning("openPort : Unable to open serial port %s",name);
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


void PosixSerialPort::run()
{
  while (THREAD_ALIVE)
  {
    numberBytesToRead = nbBytesToRead();
    if (numberBytesToRead > 0)
    {
      t_ = road_time();                     // datation
      receivedBuffer_ = new char[numberBytesToRead];
      memset(receivedBuffer_,0,numberBytesToRead);
      numberBytesRead = readBuffer(receivedBuffer_,numberBytesToRead);
      processIncomingData();
    }
    else
    {
      receivedBuffer_ = NULL;
      // todo : trouver une autre methode plus efficace que le polling et le sleep !
      usleep(1000);
    }
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
    FRAME * frame = new FRAME;
    frame->t = t_;
    frame->tr = 0;
    frame->length = numberBytesRead;
    frame->data = QByteArray(receivedBuffer_, frame->length);

    frameLock_.lock();
    dataFrameList.append( frame );
    frameLock_.unlock();

    //printf(receivedBuffer_);
    emit newDataAvailable(1);
    delete[] receivedBuffer_;
    receivedBuffer_ = NULL;
  }

  if (ringIndicatorDetected) {
    ringIndicatorDetected = false;
    FRAME * frame = new FRAME;
    frame->t = t_;
    frame->tr = 0;
    frame->length = 3;
    frame->data = frame->data = QByteArray("PPS", frame->length);

    frameLock_.lock();
    dataFrameList.append(frame);
    frameLock_.unlock();

    emit newDataAvailable(1);
  }

  // re-initialization
  t_ = 0;
  numberBytesToRead = 0;
}
