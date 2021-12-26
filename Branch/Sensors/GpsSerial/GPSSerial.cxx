//  @ Project : RADAR
//  @ File Name : sensorlrr.h
//  @ Date : 2016/12/22
//  @ Author :
//
//


#ifndef _GPSSerial_H
#define _GPSSerial_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Basis/icvStructureData.hxx"
#include "OpenICV/structure/structureFrame.h"
#include "OpenICV/structure/structure_gps.h"



#include <cstdlib>
#include <string>
#include <sstream>
#include <boost/thread/thread.hpp>
#include "NMEA0183/NMEA0183.H"

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
class GpsSerial: public icvFunction
{
public:

  GpsSerial(icv_shared_ptr<const icvMetaData> info) : icvFunction(info)
  {


    Register_Sub("SerialPort");

  };


GpsSerial() : GpsSerial(nullptr) {};

~GpsSerial(){};





int frameType(string frame)
{
  for (int i = 0; i < MAX_FRAMES; i++) {
    if (strcmp(frame.c_str(),frameTypes[i].code.c_str())==0) {
      return frameTypes[i].type;
    }
  }
  return UNKNOWN_NMEA_FRAME;
};




bool analyzeFrame(icv::buffFrame * currentFrame)
{
  // Process the remaining bytes in the current frame
    int nextByteToProcess_ = 0;
   bool startOfFrame_=false;
    bool endOfFrame_ = false;
  while(nextByteToProcess_ < currentFrame->data.length() ) 
  {
    uint8_t currentChar = currentFrame->data[nextByteToProcess_++];
    // first looking for start-of-frame
    if (!startOfFrame_ && (currentChar == '$')) {
      startOfFrame_ = true;
      endOfFrame_ = false;

      frameToDecode_.time_stamp_ = currentFrame->time_stamp_;
      frameToDecode_.data="";
    } else if (startOfFrame_ && !endOfFrame_ && (currentChar == '\n')) {
      // Looking for end-of-frame
      startOfFrame_ = false;
      endOfFrame_ = true;
      frameToDecode_.length_=frameToDecode_.data.length();
      return true;  // There is a new frame to decode
    }
    if ((startOfFrame_) && (!endOfFrame_)) {
      frameToDecode_.data.push_back(currentChar);
    }
  }
  return false; // No new frame to decode, wait for more data
};



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
};

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
    // ppsFrame.setRoadTime(lastPpsTime_);
    // *ppsFrame.getFrameData() = ppsIndex_++;
    //ppsFrame.notifyObservers();

    break;

  case TRAME_GGA_DBL:
    if (!nmea0183_.Gga.Parse(sentence)) {
      ICV_LOG_WARN<<"Failed to parse the frame " << nmea0183_.Gga.ErrorMessage;
    }
    else {
    lat_rad = nmea0183_.Gga.Position.Latitude.GetDecimalDegrees()*PI/180;
    lon_rad = nmea0183_.Gga.Position.Longitude.GetDecimalDegrees()*PI/180;
    GGAtemp.H = gmtime ( &nmea0183_.Gga.Time )->tm_hour;
    GGAtemp.Mi = gmtime ( &nmea0183_.Gga.Time )->tm_min;
    GGAtemp.S =  gmtime ( &nmea0183_.Gga.Time )->tm_sec;
    GGAtemp.Ms =  nmea0183_.Gga.msec ;
    GGAtemp.lon = lon_rad;
    GGAtemp.lat = lat_rad;
    GGAtemp.ind_qualite = nmea0183_.Gga.GPSQuality;
    GGAtemp.nb_sat = nmea0183_.Gga.NumberOfSatellitesInUse;
    GGAtemp.hdop = nmea0183_.Gga.HorizontalDilutionOfPrecision;
    GGAtemp.alt_msl = nmea0183_.Gga.AntennaAltitudeMeters;
    GGAtemp.d_geoidal = nmea0183_.Gga.GeoidalSeparationMeters;
    GGAtemp.age = nmea0183_.Gga.AgeOfDifferentialGPSDataSeconds;
    GGAtemp.dir_lat = ( (nmea0183_.Gga.Position.Latitude.Northing == North) ? 'N' : 'S' );
    GGAtemp.dir_lon = ( (nmea0183_.Gga.Position.Longitude.Easting == East) ? 'E' : 'W' );
    GGAtemp.ref_station_ID = nmea0183_.Gga.DifferentialReferenceStationID;

	  gpsall1.lon=GGAtemp.lon;
	  gpsall1.lat=GGAtemp.lat;
	  lonlattolam(gpsall1.lon, gpsall1.lat, gpsall1.x, gpsall1.y ) ;
    }
    break;

  case TRAME_GSA:
    if (!nmea0183_.Gsa.Parse(sentence))
    GSAtemp.mode_select = ((nmea0183_.Gsa.OperatingMode == GSA::Manual) ? 'M' : 'A');
    GSAtemp.mode_result = 0;
    if (nmea0183_.Gsa.FixMode == GSA::FixUnavailable)
      GSAtemp.mode_result = 1;
    if (nmea0183_.Gsa.FixMode == GSA::TwoDimensional)
      GSAtemp.mode_result = 2;
    if (nmea0183_.Gsa.FixMode == GSA::ThreeDimensional)
      GSAtemp.mode_result = 3;
    for (indexGSA = 0 ; indexGSA<12 ; indexGSA++)
      GSAtemp.SV_PRN[indexGSA] = nmea0183_.Gsa.SatelliteNumber[indexGSA];
    GSAtemp.pdop = nmea0183_.Gsa.PDOP;
    GSAtemp.hdop = nmea0183_.Gsa.HDOP;
    GSAtemp.vdop = nmea0183_.Gsa.VDOP;
 
    break;



  case TRAME_GST:
    if (!nmea0183_.Gst.Parse( sentence ))
     // qWarning("Failed to parse the frame %s\n",nmea0183_.Gst.ErrorMessage.toLatin1().data());
    GSTtemp.rms = nmea0183_.Gst.RMSvalue;
    GSTtemp.a = nmea0183_.Gst.ErrorEllipseMajor;
    GSTtemp.b = nmea0183_.Gst.ErrorEllipseMinor;
    GSTtemp.phi = nmea0183_.Gst.ErrorEllipseOrientation;
    GSTtemp.sigma_lat = nmea0183_.Gst.LatitudeError;
    GSTtemp.sigma_lon = nmea0183_.Gst.LongitudeError;
    GSTtemp.sigma_alt = nmea0183_.Gst.HeightError;
    GSTtemp.H = gmtime ( &nmea0183_.Gst.Time )->tm_hour;
    GSTtemp.Mi = gmtime ( &nmea0183_.Gst.Time )->tm_min;
    GSTtemp.S = gmtime ( &nmea0183_.Gst.Time )->tm_sec;
    GSTtemp.Ms =nmea0183_.Gst.msec;
	
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
        GSVtemp.SatellitesInView[ indexGSV ][ 0 ] = 0;
        GSVtemp.SatellitesInView[ indexGSV ][ 1 ] = 0;
        GSVtemp.SatellitesInView[ indexGSV ][ 2 ] = 0;
        GSVtemp.SatellitesInView[ indexGSV ][ 3 ] = 0;
        indexGSV++;
      }
    }
    GSVtemp.NumberOfSatellites = nmea0183_.Gsv.NumberOfSatellites;
    //GSVtemp.Totalmessages = nmea0183_.Gsv.Totalmessages;

    for  ( indexGSV=4*(nmea0183_.Gsv.message_number-1); indexGSV<=(4*nmea0183_.Gsv.message_number)-1; indexGSV++ )
    {
      GSVtemp.SatellitesInView[ indexGSV ][ 0 ] = nmea0183_.Gsv.SatellitesInView[ indexGSV ].SatelliteNumber;
      GSVtemp.SatellitesInView[ indexGSV ][ 1 ] = nmea0183_.Gsv.SatellitesInView[ indexGSV ].ElevationDegrees;
      GSVtemp.SatellitesInView[ indexGSV ][ 2 ] = nmea0183_.Gsv.SatellitesInView[ indexGSV ].AzimuthDegreesTrue;
      GSVtemp.SatellitesInView[ indexGSV ][ 3 ] = nmea0183_.Gsv.SatellitesInView[ indexGSV ].SignalToNoiseRatio;
    }

    if (nmea0183_.Gsv.Totalmessages == nmea0183_.Gsv.message_number) {
     // sendDataToServerSocket(*gsvFrame.getFrameData(),type);

		//checkedSend(GSVdata,GSVtemp);

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
     HDTtemp.DegreesTrue = nmea0183_.Hdt.DegreesTrue;

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
      RMCtemp.H = gmtime ( &nmea0183_.Rmc.Time )->tm_hour;
      RMCtemp.Mi = gmtime ( &nmea0183_.Rmc.Time )->tm_min;
      RMCtemp.S = gmtime ( &nmea0183_.Rmc.Time )->tm_sec;
      RMCtemp.Ms = nmea0183_.Rmc.msec;
      RMCtemp.AA = nmea0183_.Rmc.yy;
      RMCtemp.MM = nmea0183_.Rmc.mm;
      RMCtemp.JJ = nmea0183_.Rmc.dd;
      RMCtemp.lat = nmea0183_.Rmc.Position.Latitude.GetDecimalDegrees()*PI/180;
      RMCtemp.dir_lat = ( (nmea0183_.Rmc.Position.Latitude.Northing == North) ? 'N' : 'S');
      RMCtemp.lon = nmea0183_.Rmc.Position.Longitude.GetDecimalDegrees()*PI/180;;
      RMCtemp.dir_lon = ( (nmea0183_.Rmc.Position.Longitude.Easting == East) ? 'E' : 'W' );
      RMCtemp.magnet_var = nmea0183_.Rmc.MagneticVariation;
      RMCtemp.dir_magnet_var = ( (nmea0183_.Rmc.MagneticVariationDirection == East) ? 'E' : 'W');
      RMCtemp.mode = -1;
     // if (nmea0183_.Rmc.ModeIndication == "A")
       // RMCtemp.mode = 1;
      ///if (nmea0183_.Rmc.ModeIndication == "D")
        //RMCtemp.mode = 2;
      //if (nmea0183_.Rmc.ModeIndication == "N")
       // RMCtemp.mode = 0;
      RMCtemp.track_true_north = nmea0183_.Rmc.TrackMadeGoodDegreesTrue;
      RMCtemp.valid_data = ( (nmea0183_.Rmc.IsDataValid == True) ? 1 : 0 );
      RMCtemp.vitesse = nmea0183_.Rmc.SpeedOverGroundKnots * 1852.0 / 3600.0; // 1 knot = 1852 m/h
  
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
       VTGtemp.v = nmea0183_.Vtg.SpeedKilometersPerHour;
       VTGtemp.track_true_north = nmea0183_.Vtg.TrackDegreesTrue;
       VTGtemp.track_magnet_north = nmea0183_.Vtg.TrackDegreesMagnetic;

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
      ZDAtemp.H = gmtime(&nmea0183_.Zda.Time)->tm_hour;
      ZDAtemp.Mi = gmtime(&nmea0183_.Zda.Time)->tm_min;
      ZDAtemp.S = gmtime(&nmea0183_.Zda.Time)->tm_sec;
      ZDAtemp.Ms = nmea0183_.Zda.msec;
      ZDAtemp.AA = nmea0183_.Zda.Year;
      ZDAtemp.MM = nmea0183_.Zda.Month;
      ZDAtemp.JJ = nmea0183_.Zda.Day;
      ZDAtemp.H_offset = nmea0183_.Zda.LocalHourDeviation;
      ZDAtemp.Mi_offset = nmea0183_.Zda.LocalMinutesDeviation;

      //sendDataToServerSocket(*zdaFrame.getFrameData(), type);
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
      return -1;
  }

  return 1;
};
  virtual void Execute() override
  {
        icv::icvbuffFrame temp;
        icvSubscribe("SerialPort",&temp);
    if (analyzeFrame(temp.getvaluePtr())) 
    {
      // a new complete NMEA frame has arrived, decode it.
      type = frameType(frameToDecode_.data);
      if (type != -1) 
      {
        if (decodeFrame(type) == -1) 
        {
            ICV_LOG_WARN<<"Failed to decode the dataframe\n";
        }


      }

	  }
  
     


	};

private:

  
  static const int MaxUdpBufferSize = 1024;

    NMEA0183 nmea0183_;
	std::string portName_;
  static const int MAX_FRAMES = 9;
    struct FrameTypeMap {
    string code;
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
  { "$GPZDA", TRAME_ZDA }
};
  int type = -1;

  buffFrame frameToDecode_;
 donnees_gps gpsall1, gpsall2;
  trame_gga_dbl GGAtemp;
  trame_gsa GSAtemp;
  trame_gst GSTtemp;
  trame_gsv GSVtemp;
  trame_hdt HDTtemp;
   trame_rmc RMCtemp;
  trame_rot ROTtemp;
  trame_vtg VTGtemp;
  trame_zda ZDAtemp;
  uint32_t lastPpsTime_;
  unsigned int ppsIndex_=0;
 // bool newFrameToDecode_;
 // bool startOfFrame_;
 // bool endOfFrame_;
  //int nextByteToProcess_;
  // GpsGgaFrame ggaFrame;
  // GpsGsaFrame gsaFrame;
  // GpsGstFrame gstFrame;
  // GpsGsvFrame gsvFrame;
  // GpsHdtFrame hdtFrame;
  // GpsRmcFrame rmcFrame;
  // GpsRotFrame rotFrame;
  // GpsVtgFrame vtgFrame;
  // GpsZdaFrame zdaFrame;
  // GpsSynchro  synchroFrame;
  // GpsPps      ppsFrame;
};


ICV_REGISTER_FUNCTION(GpsSerial)

#endif 
