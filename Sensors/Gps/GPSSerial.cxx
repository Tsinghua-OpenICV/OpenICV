//  @ Project : RADAR
//  @ File Name : sensorlrr.h
//  @ Date : 2016/12/22
//  @ Author :
//
//


#ifndef _GPSEthernet_H
#define _GPSEthernet_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
#include "OpenICV/Net/icvUdpReceiverSource.h"
#include "OpenICV/Basis/icvStructureData.hxx"



#include <cstdlib>
#include <string>
#include <sstream>
#include <boost/thread/thread.hpp>
#include "NMEA0183/NMEA0183.H"
#include "PosixSerialPort.h"
#include "GpsFrames.h"

#define PI 3.1415926
#define UNKNOWN_NMEA_FRAME -1


typedef unsigned char uint8;
typedef float float32;
typedef double float64;
typedef char int8;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned int uint32;
typedef int int32;

using namespace icv;
using namespace std;
using namespace core;
using namespace icv::function;
class GpsSerial: public icvFunction
{
public:
  typedef data::icvStructureData<trame_gga_dbl>    icvGpsGgaFrame;

  GpsSerial(icv_shared_ptr<const icvMetaData> info) : icvFunction(info)
  {
    setPortCOM(_information.GetString("port").c_str());
    serialPort = new PosixSerialPort();
    if (!serialPort->openPort(portName_.c_str()))
    {
      ICV_LOG_INFO<<"Failed to open the port " << portName_;
      ICV_LOG_INFO<<"The GPS Component  didn't start";    
    }
    else
    ICV_LOG_INFO<<"GPS port open";

    int baudrate,bytesize,parity,stopbits;
    baudrate=115200;
    bytesize=8 ;
    parity=0;
    stopbits=1 ;
    if(_information.Contains("baudrate"))baudrate=_information.GetInteger("baudrate");
    if(_information.Contains("bytesize"))bytesize=_information.GetInteger("bytesize");
    if(_information.Contains("parity"))  parity  =_information.GetInteger("parity");
    if(_information.Contains("stopbits"))stopbits=_information.GetInteger("stopbits");

    serialPort->configurePort(baudrate,bytesize,parity,stopbits-1);
    nextByteToProcess_ = 0;
    currentFrame_ = NULL;
    newFrameToDecode_ = false;
    startOfFrame_ = false;
    endOfFrame_ = false;
  };


GpsSerial() : GpsSerial(nullptr) {}

void setPortCOM(const char * port)
{
  portName_ = port;
}



int frameType(const char* frame)
{
  for (int i = 0; i < MAX_FRAMES; i++) {
    if (strcmp(frame,frameTypes[i].code)==0) {
      return frameTypes[i].type;
    }
  }
  return UNKNOWN_NMEA_FRAME;
}



bool currentFrameIsPps() {
  if (currentFrame_->data == "PPS") {
    nextByteToProcess_ = 3;
    return true;
  } else {
    return false;
  }
}

/************************************************************************/
/* Reconstruct the frame starting from the received fragments           */
/************************************************************************/
bool analyzeFrame()
{
  // Process the remaining bytes in the current frame

  while(nextByteToProcess_ < strlen(currentFrame_->data)) 
  {
    char currentChar = currentFrame_->data[nextByteToProcess_++];
    // first looking for start-of-frame
    if (!startOfFrame_ && (currentChar == '$')) {
      startOfFrame_ = true;
      endOfFrame_ = false;

      frameToDecode_.t = currentFrame_->t;
      frameToDecode_.data="";
    } else if (startOfFrame_ && !endOfFrame_ && (currentChar == '\n')) {
      // Looking for end-of-frame
      startOfFrame_ = false;
      endOfFrame_ = true;
      frameToDecode_.t = 1;
      //frameToDecode_.tr = static_cast<uint32_t>(currentFrame_->t - frameToDecode_.t);
      return true;  // There is a new frame to decode
    }
    if ((startOfFrame_) && (!endOfFrame_)) {
      //frameToDecode_.data.append(currentChar);
      strcat(frameToDecode_.data,&currentChar);
    }
  }
  return false; // No new frame to decode, wait for more data
}


void unlockProcessing(int v)
{
  // new frame available
  //semaphore_.release( v );
}

void lonlattolam(double lon, double lat, double & lam93x, double & lam93y)
{
  double GRS_a = 6378137;
  double GRS_f = 1/298.257222101;
  
  double GRS_b = GRS_a*(1-GRS_f);
  double GRS_bb= GRS_b*GRS_b;
  double GRS_aa= 40680631590769.0;
  double GRS_e = sqrt((GRS_aa - GRS_bb) / (GRS_aa));
  
  double n = 0.725607765053267;
  double C = 11754255.4261;
  double XS = 700000;
  double YS = 12655612.0499;
  
  double latiso;
  latiso = atanh(sin(lat)) - GRS_e*atanh(GRS_e*sin(lat));
  double gamma;
  gamma = (lon - 0.0523598775598299)*n;
  double R;
  R = C * exp(-n*latiso);
  
  lam93x = R *sin(gamma)+XS;
  lam93y = -R *cos(gamma)+YS;
}

int decodeFrame(int type)
{
  double lat_rad = 0, lon_rad = 0;
  int indexGSV = 0;
  int indexGSA = 0;


  SENTENCE sentence;
  sentence.Sentence = frameToDecode_.data;

  switch(type)
  {
  case UNKNOWN_NMEA_FRAME:
    ICV_LOG_INFO<<"Unknown frame received !";
    break;
  case SIGNAL_PPS:
    ppsFrame.setRoadTime(lastPpsTime_);
    *ppsFrame.getFrameData() = ppsIndex_++;
    //ppsFrame.notifyObservers();

    break;

  case TRAME_GGA_DBL:
    if (!nmea0183_.Gga.Parse(sentence)) {
      ICV_LOG_WARN<<"Failed to parse the frame " << nmea0183_.Gga.ErrorMessage;
    }
    else {
    lat_rad = nmea0183_.Gga.Position.Latitude.GetDecimalDegrees()*PI/180;
    lon_rad = nmea0183_.Gga.Position.Longitude.GetDecimalDegrees()*PI/180;
    ggaFrame.getFrameData()->H = gmtime ( &nmea0183_.Gga.Time )->tm_hour;
    ggaFrame.getFrameData()->Mi = gmtime ( &nmea0183_.Gga.Time )->tm_min;
    ggaFrame.getFrameData()->S =  gmtime ( &nmea0183_.Gga.Time )->tm_sec;
    ggaFrame.getFrameData()->Ms =  nmea0183_.Gga.msec ;
    ggaFrame.getFrameData()->lon = lon_rad;
    ggaFrame.getFrameData()->lat = lat_rad;
    ggaFrame.getFrameData()->ind_qualite = nmea0183_.Gga.GPSQuality;
    ggaFrame.getFrameData()->nb_sat = nmea0183_.Gga.NumberOfSatellitesInUse;
    ggaFrame.getFrameData()->hdop = nmea0183_.Gga.HorizontalDilutionOfPrecision;
    ggaFrame.getFrameData()->alt_msl = nmea0183_.Gga.AntennaAltitudeMeters;
    ggaFrame.getFrameData()->d_geoidal = nmea0183_.Gga.GeoidalSeparationMeters;
    ggaFrame.getFrameData()->age = nmea0183_.Gga.AgeOfDifferentialGPSDataSeconds;
    ggaFrame.getFrameData()->dir_lat = ( (nmea0183_.Gga.Position.Latitude.Northing == North) ? 'N' : 'S' );
    ggaFrame.getFrameData()->dir_lon = ( (nmea0183_.Gga.Position.Longitude.Easting == East) ? 'E' : 'W' );
    ggaFrame.getFrameData()->ref_station_ID = nmea0183_.Gga.DifferentialReferenceStationID;
    ggaFrame.setRoadTime(frameToDecode_.t);
	  memcpy(&GGAtemp,ggaFrame.getFrameData(),sizeof(trame_gga_dbl));
	  gpsall1.lon=GGAtemp.lon;
	  gpsall1.lat=GGAtemp.lat;
	  lonlattolam(gpsall1.lon, gpsall1.lat, gpsall1.x, gpsall1.y ) ;
    }
    break;

  case TRAME_GSA:
    if (!nmea0183_.Gsa.Parse(sentence))
    gsaFrame.getFrameData()->mode_select = ((nmea0183_.Gsa.OperatingMode == GSA::Manual) ? 'M' : 'A');
    gsaFrame.getFrameData()->mode_result = 0;
    if (nmea0183_.Gsa.FixMode == GSA::FixUnavailable)
      gsaFrame.getFrameData()->mode_result = 1;
    if (nmea0183_.Gsa.FixMode == GSA::TwoDimensional)
      gsaFrame.getFrameData()->mode_result = 2;
    if (nmea0183_.Gsa.FixMode == GSA::ThreeDimensional)
      gsaFrame.getFrameData()->mode_result = 3;
    for (indexGSA = 0 ; indexGSA<12 ; indexGSA++)
      gsaFrame.getFrameData()->SV_PRN[indexGSA] = nmea0183_.Gsa.SatelliteNumber[indexGSA];
    gsaFrame.getFrameData()->pdop = nmea0183_.Gsa.PDOP;
    gsaFrame.getFrameData()->hdop = nmea0183_.Gsa.HDOP;
    gsaFrame.getFrameData()->vdop = nmea0183_.Gsa.VDOP;
    gsaFrame.setRoadTime(frameToDecode_.t);

	  memcpy(&GSAtemp,gsaFrame.getFrameData(),sizeof(trame_gsa));
    break;



  case TRAME_GST:
    if (!nmea0183_.Gst.Parse( sentence ))
     // qWarning("Failed to parse the frame %s\n",nmea0183_.Gst.ErrorMessage.toLatin1().data());
    gstFrame.getFrameData()->rms = nmea0183_.Gst.RMSvalue;
    gstFrame.getFrameData()->a = nmea0183_.Gst.ErrorEllipseMajor;
    gstFrame.getFrameData()->b = nmea0183_.Gst.ErrorEllipseMinor;
    gstFrame.getFrameData()->phi = nmea0183_.Gst.ErrorEllipseOrientation;
    gstFrame.getFrameData()->sigma_lat = nmea0183_.Gst.LatitudeError;
    gstFrame.getFrameData()->sigma_lon = nmea0183_.Gst.LongitudeError;
    gstFrame.getFrameData()->sigma_alt = nmea0183_.Gst.HeightError;
    gstFrame.getFrameData()->H = gmtime ( &nmea0183_.Gst.Time )->tm_hour;
    gstFrame.getFrameData()->Mi = gmtime ( &nmea0183_.Gst.Time )->tm_min;
    gstFrame.getFrameData()->S = gmtime ( &nmea0183_.Gst.Time )->tm_sec;
    gstFrame.getFrameData()->Ms =nmea0183_.Gst.msec;
    gstFrame.setRoadTime(frameToDecode_.t);
	
	   memcpy(&GSTtemp,gstFrame.getFrameData(),sizeof(trame_gst));
		//checkedSend(GSTdata,GSTtemp);
    //sendDataToServerSocket(*gstFrame.getFrameData(),type);
    ///gstFrame.notifyObservers();


    // if ( (isRecording()) && (gstRecording) ) {
		//   if (!gsthdFile.isOpen())
		// 	  gsthdFile.open((char *)(prefix+name() + "_gst.dbt").toLatin1().data(),WriteMode,TRAME_GST,sizeof(trame_gst));
    //   if ( !gsthdFile.writeRecord(frameToDecode_.t, frameToDecode_.tr,(char *) gstFrame.getFrameData(),sizeof(trame_gst)) )
    //     qWarning("Failed to record GST data ...\n");
    // }
    break;

  case TRAME_GSV:
    indexGSV = 0;
    if (!nmea0183_.Gsv.Parse( sentence )) {
   //   qWarning("Failed to parse the frame %s\n",nmea0183_.Gsv.ErrorMessage.toLatin1().data());
      break;
    }
    // it's a new frame, reset stored value in case of the number of satellites
    // in view has decreased
    if (nmea0183_.Gsv.message_number == 1)
    {
      while (indexGSV < 36)
      {
        gsvFrame.getFrameData()->SatellitesInView[ indexGSV ][ 0 ] = 0;
        gsvFrame.getFrameData()->SatellitesInView[ indexGSV ][ 1 ] = 0;
        gsvFrame.getFrameData()->SatellitesInView[ indexGSV ][ 2 ] = 0;
        gsvFrame.getFrameData()->SatellitesInView[ indexGSV ][ 3 ] = 0;
        indexGSV++;
      }
    }
    gsvFrame.getFrameData()->NumberOfSatellites = nmea0183_.Gsv.NumberOfSatellites;
    //gsvFrame.getFrameData()->Totalmessages = nmea0183_.Gsv.Totalmessages;

    for  ( indexGSV=4*(nmea0183_.Gsv.message_number-1); indexGSV<=(4*nmea0183_.Gsv.message_number)-1; indexGSV++ )
    {
      gsvFrame.getFrameData()->SatellitesInView[ indexGSV ][ 0 ] = nmea0183_.Gsv.SatellitesInView[ indexGSV ].SatelliteNumber;
      gsvFrame.getFrameData()->SatellitesInView[ indexGSV ][ 1 ] = nmea0183_.Gsv.SatellitesInView[ indexGSV ].ElevationDegrees;
      gsvFrame.getFrameData()->SatellitesInView[ indexGSV ][ 2 ] = nmea0183_.Gsv.SatellitesInView[ indexGSV ].AzimuthDegreesTrue;
      gsvFrame.getFrameData()->SatellitesInView[ indexGSV ][ 3 ] = nmea0183_.Gsv.SatellitesInView[ indexGSV ].SignalToNoiseRatio;
    }

    if (nmea0183_.Gsv.Totalmessages == nmea0183_.Gsv.message_number) {
     // sendDataToServerSocket(*gsvFrame.getFrameData(),type);

	   memcpy(&GSVtemp,gsvFrame.getFrameData(),sizeof(trame_gsv));
		//checkedSend(GSVdata,GSVtemp);

      gsvFrame.setRoadTime(frameToDecode_.t);
    //  / gsvFrame.notifyObservers();
    }
    // if ( (isRecording()) && (gsvRecording) && (nmea0183_.Gsv.Totalmessages == nmea0183_.Gsv.message_number) )
    // {
		// if (!gsvhdFile.isOpen())
    //     gsvhdFile.open((char *)(prefix+name() + "_gsv.dbt").toLatin1().data(),WriteMode,TRAME_GSV,sizeof(trame_gsv));
		// if ( gsvhdFile.writeRecord(frameToDecode_.t, frameToDecode_.tr,(char *) gsvFrame.getFrameData(),sizeof(trame_gsv)) == 0)
    //     qWarning("Failed to record GSV data ...\n");
    // }
    break;

    case TRAME_HDT:
      if (!nmea0183_.Hdt.Parse( sentence ))
     //   qWarning("Failed to parse the frame %s\n",nmea0183_.Hdt.ErrorMessage.toLatin1().data());
      hdtFrame.getFrameData()->DegreesTrue = nmea0183_.Hdt.DegreesTrue;
      hdtFrame.setRoadTime(frameToDecode_.t);
	 
	  	  memcpy(&HDTtemp,hdtFrame.getFrameData(),sizeof(trame_hdt));
	//	checkedSend(HDTdata,HDTtemp);
    //  sendDataToServerSocket(hdtFrame,type);
     // hdtFrame.notifyObservers();

      // if ( (isRecording()) && (hdtRecording) )
      // {
		  // if (!hdthdFile.isOpen())
			//   hdthdFile.open((char *)(prefix+name() + "_hdt.dbt").toLatin1().data(),WriteMode,TRAME_HDT,sizeof(trame_hdt));
      // if ( hdthdFile.writeRecord(frameToDecode_.t, frameToDecode_.tr,(char *) hdtFrame.getFrameData(),sizeof(trame_hdt)) == 0)
      //     qWarning("Failed to record HDT data ...\n");
      // }
      break;

    case TRAME_RMC:
      if (!nmea0183_.Rmc.Parse( sentence ))
       // qWarning("Failed to parse the frame %s\n",nmea0183_.Rmc.ErrorMessage.toLatin1().data());
      rmcFrame.getFrameData()->H = gmtime ( &nmea0183_.Rmc.Time )->tm_hour;
      rmcFrame.getFrameData()->Mi = gmtime ( &nmea0183_.Rmc.Time )->tm_min;
      rmcFrame.getFrameData()->S = gmtime ( &nmea0183_.Rmc.Time )->tm_sec;
      rmcFrame.getFrameData()->Ms = nmea0183_.Rmc.msec;
      rmcFrame.getFrameData()->AA = nmea0183_.Rmc.yy;
      rmcFrame.getFrameData()->MM = nmea0183_.Rmc.mm;
      rmcFrame.getFrameData()->JJ = nmea0183_.Rmc.dd;
      rmcFrame.getFrameData()->lat = nmea0183_.Rmc.Position.Latitude.GetDecimalDegrees()*PI/180;
      rmcFrame.getFrameData()->dir_lat = ( (nmea0183_.Rmc.Position.Latitude.Northing == North) ? 'N' : 'S');
      rmcFrame.getFrameData()->lon = nmea0183_.Rmc.Position.Longitude.GetDecimalDegrees()*PI/180;;
      rmcFrame.getFrameData()->dir_lon = ( (nmea0183_.Rmc.Position.Longitude.Easting == East) ? 'E' : 'W' );
      rmcFrame.getFrameData()->magnet_var = nmea0183_.Rmc.MagneticVariation;
      rmcFrame.getFrameData()->dir_magnet_var = ( (nmea0183_.Rmc.MagneticVariationDirection == East) ? 'E' : 'W');
      rmcFrame.getFrameData()->mode = -1;
     // if (nmea0183_.Rmc.ModeIndication == "A")
       // rmcFrame.getFrameData()->mode = 1;
      ///if (nmea0183_.Rmc.ModeIndication == "D")
        //rmcFrame.getFrameData()->mode = 2;
      //if (nmea0183_.Rmc.ModeIndication == "N")
       // rmcFrame.getFrameData()->mode = 0;
      rmcFrame.getFrameData()->track_true_north = nmea0183_.Rmc.TrackMadeGoodDegreesTrue;
      rmcFrame.getFrameData()->valid_data = ( (nmea0183_.Rmc.IsDataValid == True) ? 1 : 0 );
      rmcFrame.getFrameData()->vitesse = nmea0183_.Rmc.SpeedOverGroundKnots * 1852.0 / 3600.0; // 1 knot = 1852 m/h
      rmcFrame.setRoadTime(frameToDecode_.t);
	  	  memcpy(&RMCtemp,rmcFrame.getFrameData(),sizeof(trame_rmc));
		// checkedSend(RMCdata,RMCtemp);
	  
    //   sendDataToServerSocket(*rmcFrame.getFrameData(),type);
    //   rmcFrame.notifyObservers();

    //   if ( (isRecording()) && (rmcRecording) )
    //   {
    //     if (!rmchdFile.isOpen())
    //       rmchdFile.open((char *)(prefix+name() + "_rmc.dbt").toLatin1().data(),WriteMode,TRAME_RMC,sizeof(trame_rmc));
    //     if (rmchdFile.writeRecord(frameToDecode_.t ,frameToDecode_.tr,(char *) rmcFrame.getFrameData(),sizeof(trame_rmc)) == 0)
    //       qWarning("Failed to record RMC data ...\n");
    //   }
      break;

    // case TRAME_ROT:
    //   if (!nmea0183_.Rot.Parse( sentence ))
    //     qWarning("Failed to parse the frame %s\n",nmea0183_.Rot.ErrorMessage.toLatin1().data());
    //   rotFrame.getFrameData()->RateOfTurn = nmea0183_.Rot.RateOfTurn;
    //   rotFrame.getFrameData()->valid_data = ( (nmea0183_.Rot.IsDataValid == True) ? 1 : 0 );
    //   rotFrame.setRoadTime(frameToDecode_.t);
	  // memcpy(&ROTtemp,rotFrame.getFrameData(),sizeof(trame_rot));
		// checkedSend(ROTdata,ROTtemp);
	  
    //   sendDataToServerSocket(*rotFrame.getFrameData(),type);
    //   rotFrame.notifyObservers();

    //   if ( (isRecording()) && (rotRecording) )
    //   {
    //     if (!rothdFile.isOpen())
    //       rothdFile.open((char *)(prefix+name() + "_rot.dbt").toLatin1().data(),WriteMode,TRAME_ROT,sizeof(trame_rot));
    //     if ( rothdFile.writeRecord(frameToDecode_.t ,frameToDecode_.tr,(char *) rotFrame.getFrameData(),sizeof(trame_rot)) == 0)
    //       qWarning("Failed to record ROT data ...\n");
    //   }
    //   break;

    case TRAME_VTG:
      if (!nmea0183_.Vtg.Parse( sentence )) {
    // /    LOG_WARN("Failed to parse the frame " << nmea0183_.Vtg.ErrorMessage);
      } else {
        vtgFrame.getFrameData()->v = nmea0183_.Vtg.SpeedKilometersPerHour;
        vtgFrame.getFrameData()->track_true_north = nmea0183_.Vtg.TrackDegreesTrue;
        vtgFrame.getFrameData()->track_magnet_north = nmea0183_.Vtg.TrackDegreesMagnetic;
        vtgFrame.setRoadTime(frameToDecode_.t);
		memcpy(&VTGtemp,vtgFrame.getFrameData(),sizeof(trame_vtg));

		gpsall2.a=VTGtemp.v;
		gpsall2.b=VTGtemp.track_true_north;
		gpsall2.phi=VTGtemp.track_magnet_north;
		// checkedSend(VTGdata,VTGtemp);
		// checkedSend(VTGdatadyna,gpsall2);
    //     sendDataToServerSocket(vtgFrame,type);
    //     vtgFrame.notifyObservers();

        // if (isRecording() && vtgRecording) {
        //   if (!vtghdFile.isOpen())
        //     vtghdFile.open((char *)(prefix+name() + "_vtg.dbt").toLatin1().data(),WriteMode,TRAME_VTG,sizeof(trame_vtg));
		    //   if ( vtghdFile.writeRecord(frameToDecode_.t, frameToDecode_.tr,(char *) vtgFrame.getFrameData(),sizeof(trame_vtg)) == 0)
        //     qWarning("Failed to record VTG data ...\n");
        // }
      }
      break;

    case TRAME_ZDA:
      if (!nmea0183_.Zda.Parse( sentence )) {
       // LOG_WARN("Failed to parse the frame " << nmea0183_.Zda.ErrorMessage);
      }
      zdaFrame.getFrameData()->H = gmtime(&nmea0183_.Zda.Time)->tm_hour;
      zdaFrame.getFrameData()->Mi = gmtime(&nmea0183_.Zda.Time)->tm_min;
      zdaFrame.getFrameData()->S = gmtime(&nmea0183_.Zda.Time)->tm_sec;
      zdaFrame.getFrameData()->Ms = nmea0183_.Zda.msec;
      zdaFrame.getFrameData()->AA = nmea0183_.Zda.Year;
      zdaFrame.getFrameData()->MM = nmea0183_.Zda.Month;
      zdaFrame.getFrameData()->JJ = nmea0183_.Zda.Day;
      zdaFrame.getFrameData()->H_offset = nmea0183_.Zda.LocalHourDeviation;
      zdaFrame.getFrameData()->Mi_offset = nmea0183_.Zda.LocalMinutesDeviation;

      //sendDataToServerSocket(*zdaFrame.getFrameData(), type);
      zdaFrame.setRoadTime(frameToDecode_.t);
      //zdaFrame.notifyObservers();

      // if ( (isRecording()) && (zdaRecording) ) {
		  // if (!zdahdFile.isOpen())
      //     zdahdFile.open((char *)(prefix+name() + "_zda.dbt").toLatin1().data(),WriteMode,TRAME_ZDA,sizeof(trame_zda));
      // if ( zdahdFile.writeRecord(frameToDecode_.t, frameToDecode_.tr,(char *) zdaFrame.getFrameData(),sizeof(trame_zda)) == 0) {
      //       LOG_WARN("Failed to record ZDA data ...");
      //     }
      // }



      break;

    default:
      return 0;
  }

  return 1;
}

  virtual void Execute(std::vector<icvNodeInput*>& inputPorts, std::vector<icvNodeOutput*>& outputPorts) override
  {

  buff_Input(inputPorts);
buff_Output(outputPorts);
 if ((currentFrame_ != NULL) && (nextByteToProcess_ >= strlen(currentFrame_->data))) {
      delete currentFrame_;
      currentFrame_ = NULL;
    }

    // If there is no current frame, get one. Wait for one if none is available.
    if (currentFrame_ == NULL) {
      if  ((currentFrame_ = serialPort->getNextFrame()) == NULL) 
		{

    	if (currentFrameIsPps()) {lastPpsTime_ = currentFrame_->t;
      decodeFrame(SIGNAL_PPS);
    } else if (analyzeFrame()) {
      // a new complete NMEA frame has arrived, decode it.
      type = frameType(frameToDecode_.data);
      if (type != -1) {
        if (decodeFrame(type) == -1) {
            ICV_LOG_WARN<<"Failed to decode the dataframe\n";
        }

        //send_Output<icvGpsGgaFrame>(0)=ggaFrame;
        //Send_Out(ggaFrame,0);
      }

	}
        // Should never get to this point as the semaphore counts the frames, unless
        // the thread is being stopped.
      } else {
        nextByteToProcess_ = 0;
      }
  }

    // Check if we got a PPS frame

	};

private:

  
  static const int MaxUdpBufferSize = 1024;

    NMEA0183 nmea0183_;
	PosixSerialPort * serialPort;
	std::string portName_;
  static const int MAX_FRAMES = 9;
    struct FrameTypeMap {
    char *code;
    int type;
  };

  enum Trame
{
  SIGNAL_PPS =0,
  TRAME_GGA_DBL=1,
  TRAME_GSA =2,
  TRAME_HDT =3,
  TRAME_GST =4,
  TRAME_GSV=5,
  TRAME_RMC =6,
  TRAME_VTG =7,
  TRAME_ZDA =8
};

 FrameTypeMap frameTypes[MAX_FRAMES]={
  { "PPS", SIGNAL_PPS },
  { "$GPGGA", TRAME_GGA_DBL},
  { "$GPGSA", TRAME_GSA },
  { "$GPHDT", TRAME_HDT },
  { "$GPGST", TRAME_GST },
  { "$GPGSV", TRAME_GSV },
  { "$GPRMC", TRAME_RMC },
  { "$GPVTG", TRAME_VTG },
  { "$GPZDA", TRAME_ZDA },
};
  int type = -1;
  FRAME* currentFrame_;

  FRAME frameToDecode_;
 donnees_gps gpsall1, gpsall2;
  trame_gga_dbl GGAtemp;
  trame_gsa GSAtemp;
  trame_gst GSTtemp;
  trame_gsv GSVtemp;
  trame_hdt HDTtemp;
   trame_rmc RMCtemp;
  trame_rot ROTtemp;
  trame_vtg VTGtemp;
  uint32_t lastPpsTime_;
  unsigned int ppsIndex_=0;
  bool newFrameToDecode_;
  bool startOfFrame_;
  bool endOfFrame_;
  int nextByteToProcess_;
  GpsGgaFrame ggaFrame;
  GpsGsaFrame gsaFrame;
  GpsGstFrame gstFrame;
  GpsGsvFrame gsvFrame;
  GpsHdtFrame hdtFrame;
  GpsRmcFrame rmcFrame;
  GpsRotFrame rotFrame;
  GpsVtgFrame vtgFrame;
  GpsZdaFrame zdaFrame;
  GpsSynchro  synchroFrame;
  GpsPps      ppsFrame;
};


ICV_REGISTER_FUNCTION(GpsSerial)

#endif 
