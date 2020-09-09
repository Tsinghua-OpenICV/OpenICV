/*********************************************************************
// created:    2006/06/20 - 16:50
// filename:   GyroComponent.h
//
// author:     Gerald Dherbomez
// 
// version:    $Id: gyroComponent.h 1188 2012-07-06 13:24:02Z kurdejma $
//
// purpose:    class used to get the data from a gyrometer via a 
//             serial link. 
//             Actual supporting sensors : 
//              - KVH Ecore2000
//
// todo:        replace the static shared memory by the class ShMem
*********************************************************************/

#ifndef GYROCOMPONENT_H
#define GYROCOMPONENT_H

#include "kernel/componentBase.h"
#include "driver/win32SerialPort.h"
#include "structure/structure_gyro.h"
#include "DbitePlayer/SensorTcpServer.h"
#include "XbowComponentConfig.h"

class XBOWCOMPONENT_API GyroComponent
        : public Win32SerialPort
        , public ComponentBase
{
  Q_OBJECT

public:
	GyroComponent(QString name);
	~GyroComponent();

  virtual void stopActivity(); /*!< to stop the processing thread */
  virtual void startActivity(); /*!< to start the processing thread */
  virtual ComponentBase::COMPONENT_CONFIGURATION configureComponent(XmlComponentConfig config); 


public slots:
  void decodeFrame(int i); 
	
  
private:
  void parseEcore2000(const char * buf); 
  bool analyzeFrame(); 

  char * currentDataFrame_; 
  int currentDataFrameLength_; 
  road_time_t currentRoadtime_; 
  road_timerange_t currentTimerange_; 
 
  char frameToDecode_[8]; 
  int frameToDecodeLength_; 
  road_time_t timeOfFrameToDecode_; 
  road_time_t timeOfFrameToDecodeOld_; // needed to do the high frequency integration of the rate
  road_timerange_t timerangeOfFrameToDecode_; 
  bool newFrameToDecode_; 

  char * restOfFrame_; 
  int restOfFrameLength_; 

  // SOF = start of frame
  // EOF = end of frame
  bool sof_, eof_;

  // 
  float speed;
  float angle;
  hdfile_t * gyrohdFile; 
  data_gyroKVH gyroData; 

  // a pointer to the TCP server that send data on the network
  SensorTcpServer* tcpServer_; 
};

#endif // GYROCOMPONENT_H
