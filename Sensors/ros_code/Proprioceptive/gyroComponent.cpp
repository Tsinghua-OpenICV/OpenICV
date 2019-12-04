/*********************************************************************
// created:     2006/06/20 - 16:49
// filename:    GyroComponent.cpp
//
// author:      Gerald Dherbomez
//              Copyright Heudiasyc UMR UTC/CNRS 6599
// 
// version:     $Id: gyroComponent.cpp 474 2008-05-21 12:51:07Z gdherbom $
//
// purpose:     Interfacing and decoding of the gyrometer data
*********************************************************************/


#include "sensor/GyroComponent.h"
#include "kernel/ComponentFactory.h"

// Pour MobiVIP - Stage de Pierre-Michel
//#include "PositioningProvider/gyro_transmis.h"

HANDLE gyro;
HANDLE hShmGyro;
void* p2Buf;
int modulo; 
// Prévoir une interface de visualisation minimale pour le composant avec : 
// - la donnée acquise
// - l'état du composant


// Construct the factory
ComponentFactory<GyroComponent>* factory = new ComponentFactory<GyroComponent>("GyroComponent"); 



GyroComponent::GyroComponent(QString name) : Win32SerialPort(name), ComponentBase(name) 
{
  if (!connect(this,SIGNAL(newDataAvailable(int)), this, SLOT(decodeFrame(int))))
    qWarning("Failed to connect SIGNAL(newDataAvailable(int)) with SLOT(unlockProcessing(int)\n" ); 
  
  speed = 0.0;
  angle = 0.0;
  tcpServer_ = NULL; 
}


GyroComponent::~GyroComponent()
{
}


void GyroComponent::startActivity()
{
  sof_ = FALSE; 
  eof_ = FALSE; 
  modulo = 0; 

  restOfFrameLength_ = 0; 
  currentDataFrameLength_ = 0; 
  frameToDecodeLength_ = 0; 

  speed = 0.0;
  angle = 0.0;

  
  // Pour MobiVIP - Stage de Pierre-Michel
  hShmGyro =	CreateFileMapping (
			INVALID_HANDLE_VALUE, 
			NULL, 
			PAGE_READWRITE,
			0, 
			sizeof(donnees_gyro),
			"ShmGyro") ;
  gyro = CreateEvent(NULL, false, false, "gyro");
	/* map the shared memory */
	p2Buf = MapViewOfFile(hShmGyro, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(donnees_gyro)) ;
	if (p2Buf == NULL) {
		fprintf(stderr, "gyro.cpp: error while mapping memory: %d\n", GetLastError()) ;
		ExitThread(0) ;
	}

  openPort( param.getProperty("port").toLatin1() );  

  configurePort(param.getProperty("baudrate").toLong(), 
    param.getProperty("bytesize").toUInt(),
    param.getProperty("parity").at(0).toLatin1(),
    param.getProperty("stopbits").toUInt()); 

  //setMode(FILE_FLAG_OVERLAPPED); 
  setMode(0); 

  gyrohdFile = inithdFile((char *)(ComponentBase::componentName + ".dbt").constData(), GYRO_KVH, sizeof(gyroData) ); 
  Win32SerialPort::THREAD_ALIVE = TRUE; 
  start(); 

  timeOfFrameToDecodeOld_ = road_time(); 
}


void GyroComponent::stopActivity()
{
  Win32SerialPort::THREAD_ALIVE = FALSE; 
  if (!wait(2000))
  {
    terminate(); 
    qDebug("The Win32SerialPort thread blocks anormally, it has been killed !!");
  }
  if ( !closePort() ) 
    qDebug("Failed to close the port"); 
  else
    qDebug("The port is closed"); 

  close_hdfile(gyrohdFile); 
}


ComponentBase::COMPONENT_CONFIGURATION GyroComponent::configureComponent(XmlComponentConfig config)
{
  tcpServer_ = (SensorTcpServer*)ComponentManager::create()->getComponent("cvisServer");
  if (!tcpServer_)
    qDebug() << ComponentBase::componentName << " cvisServer is not present, data won't be sent on the network";

  return ComponentBase::CONFIGURED_OK;
}


void GyroComponent::decodeFrame(int i)
{
  FRAME* currentFrame; 

  // get the frame and remove it in the list
  currentFrame = firstFrame(); 
  
  currentDataFrame_ = new char[currentFrame->length]; 
  memcpy(currentDataFrame_, currentFrame->data, currentFrame->length); 
  currentDataFrameLength_ = currentFrame->length; 
  currentRoadtime_ = currentFrame->t;
  currentTimerange_ = currentFrame->tr;    

  removeFirstFrame(); 
  
  // reconstruct the frame with fragmented received frames
  analyzeFrame(); 
  int modulo = 0;
  while (newFrameToDecode_)
  {
    // a new complete gyro frame is arrived    
    parseEcore2000( frameToDecode_ );

    // Pour MobiVIP - Stage de Pierre-Michel
    
 		CopyMemory(p2Buf, (void*)(&(gyroData.psi_s)), sizeof(data_gyroKVH));
    SetEvent(gyro);

    // send gyro to the CVIS server
    if (tcpServer_)
      tcpServer_->cs->setgyrokvh(gyroData,timeOfFrameToDecode_);

    
    if (modulo)
      //printf("GYRO : %f\n",gyroData.psi_s);
    
    modulo++;



    // record data in a .dbt file
    write_hdfile(gyrohdFile, &gyroData, timeOfFrameToDecode_, timerangeOfFrameToDecode_, sizeof(gyroData));

    currentDataFrameLength_ = 0; 
    delete[] currentDataFrame_; 
    currentDataFrame_ = NULL; 
    newFrameToDecode_ = false;
    // we do a new round to see if there are again data to decode
    analyzeFrame();
  }
  
  currentRoadtime_ = 0; 
  currentTimerange_ = 0;  
  
}


bool GyroComponent::analyzeFrame()
{    
  int lengthOfFrame = restOfFrameLength_ + currentDataFrameLength_;
  char * buffer = new char[lengthOfFrame]; 
  
  // reconstruct the complete frame with the old no-decoded data 
  // and the new incoming data
  int i; 
  for ( i = 0 ; i < restOfFrameLength_ ; ++i )
    buffer[i] = restOfFrame_[i]; 
  for ( i = 0 ; i < currentDataFrameLength_ ; ++i )
    buffer[i + restOfFrameLength_] = currentDataFrame_[i];
  
  if (restOfFrameLength_ != 0)
  {
    restOfFrameLength_ = 0; 
    delete[] restOfFrame_; 
    restOfFrame_ = NULL; 
  }
    
  for ( i = 0 ; i < lengthOfFrame ; i++)
  {
    /*    
    printf("byte=0x%X ",(buffer[i] & 0xFF));
    if ((buffer[i] & 0x80) == 0x80)
      printf("OUI "); 
    else
      printf("NON "); 
    */
    
    // first looking for SOF
    if (!sof_)
    { 
      //qDebug("%x", buffer.at(i).latin1()); 
      if (buffer[i] & 0x80)
      {
        sof_ = true; 
        timeOfFrameToDecode_ = currentRoadtime_;         // datation of tmin
      }
    }
    
    if ( (sof_) && (!eof_) )
    {
      frameToDecode_[frameToDecodeLength_] = buffer[i]; 
      frameToDecodeLength_++;
    }

    // then looking for EOF
    if (!eof_)
    {
      if (8 - frameToDecodeLength_ == 0)
      {
        eof_ = true; 
        // A vérifier
        currentTimerange_ = (road_timerange_t) (currentRoadtime_ - timeOfFrameToDecode_);    // datation of timerange
        timerangeOfFrameToDecode_ = currentTimerange_; 
        restOfFrame_ = new char[lengthOfFrame -1 - i]; 
        frameToDecodeLength_ = 0; 
      }
    }
    else
    {
      restOfFrame_[restOfFrameLength_] = buffer[i]; 
      restOfFrameLength_++; 
    }
  }
  
  if ( (sof_) && (eof_) )
  {
    sof_ = false; 
    eof_ = false; 
    newFrameToDecode_ = true; 
  }

  delete[] buffer; 
  buffer = NULL; 
  return newFrameToDecode_; 
}



void GyroComponent::parseEcore2000(const char * buf)
{
  unsigned short rate = 0; 
  unsigned char rateHi = 0;
  unsigned char rateLo = 0;

  rateLo = ( buf[7] ) | ( ( buf[6] << 7 ) & 0x80 ); 
  rateHi = ( ( buf[6] >> 1 ) & 0x3F ) | ( (buf[5] << 6) & 0xC0 );
  rate = rateLo | (rateHi << 8);

  if (rate & 0x8000)
   speed = - 0.000915 * ( ( (~rate) + 1 ) & 0xFFFF ); 
  else 
    speed = 0.000915 * rate; 

  angle = angle + speed * (__int64)(timeOfFrameToDecode_ - timeOfFrameToDecodeOld_)/1e6;  
  timeOfFrameToDecodeOld_ = timeOfFrameToDecode_; 

  gyroData.p_psi_s = speed * PACPUS_PI / 180; 
  gyroData.psi_s = angle * PACPUS_PI / 180; 

  if (modulo%50 == 0)
  {
    printf ("\n\nvitesse = %.4f angle = %.2f \n\n", speed, angle); 
    modulo = 0; 
  }
  modulo++; 

  
}