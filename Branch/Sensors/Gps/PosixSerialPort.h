/*********************************************************************
// created:    2011/05/18
// filename:   PosixSerialPort.h
//
// author:     Gerald Dherbomez
// 
// version:    $Id: $
//
// purpose:    This class declares a unix (32bits) driver for serial port
*********************************************************************/



#ifndef _PosixSerialPort_H_
#define _PosixSerialPort_H_


#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
//#include "OpenICV/Net/icvUdpReceiverSource.h"
#include "OpenICV/Core/icvConfig.h"
#include "OpenICV/Core/icvTime.h"
#include "AbstractSerialPort.h"
#include "dataFrame.h"
#include <vector>
#include <boost/atomic.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <array>
#define PORT_NAME_MAXIMUM_LENGTH 20
namespace icv{


typedef struct {
  std::int32_t t; 
  std::int64_t tr; 
  unsigned long length;
  char* data;
}FRAME; 


class PosixSerialPort : public AbstractSerialPort
{
      
public:
  PosixSerialPort(); 
  ~PosixSerialPort(); 
  
  /*!< return the number of items in the list */
  int numberOfFrames();  
  
  /*!< return the first dataFrame in the list */
  FRAME* firstFrame(); 

  /*!< return the next dataFrame in the list and removes it */
  FRAME* getNextFrame();

  /*!< remove the first dataFrame in the list */
  int removeFirstFrame();

  /*!< open the port 'name'
  return true if success 
  */
  bool openPort(const char * name); 

  /*!< close the port
  return true if success 
  */
  int closePort();   
  
  /*!< configure the port 
  return true if success 
  */
  int configurePort(long baudRate, int byteSize, char parity, int stopBits); 
  
  /*!< set the mode of usage of the port 
  
  */
//  void setMode(const DWORD overlappedModeAttribute = 0); // FILE_FLAG_OVERLAPPED
    

  std::list<FRAME*> dataFrameList; 
  bool THREAD_ALIVE; 
  char portName[PORT_NAME_MAXIMUM_LENGTH+1]; 
  
  void newDataAvailable(int); 


private:  
//  void setPortName(const char * name); 
//  bool setPort(long baudrate,char parity,int byteSize,int stopBits ); 
  
  //void processIncomingEvent(); 
  void processIncomingData();   
  int readBuffer(char *buffer, int maxLength); 
  int nbBytesToRead(); 

  void run(); 
  
  int handlePort; 

  /*
  DWORD overlappedMode; 
  OVERLAPPED overlappedStructure; 
  COMMCONFIG commConfigStructure; 
  COMMTIMEOUTS commTimeoutsStructure; 
  UINT inputBufferSize;
  UINT outputBufferSize;

  DWORD baudRate_; 
  BYTE byteSize_;
  BYTE parity_;
  BYTE stopBits_;
*/
  /*
  DWORD evtMask; 
  COMSTAT comStat;
  DWORD comErr;
  DWORD comOperationStatus; 
*/

  char * receivedBuffer_; 
  int numberBytesToRead;
  int numberBytesRead; 
  bool ringIndicatorDetected;

  // local variables
  int32_t t_;

  boost::mutex frameLock_; 
};
};

    
#endif
