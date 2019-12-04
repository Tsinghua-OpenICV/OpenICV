#include "nmea0183.h"
//#pragma hdrstop

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: gga.cpp $
** $Revision: 6 $
** $Modtime: 10/12/98 6:39a $
*/



GGA::GGA()
{
   Mnemonic = "GGA";
   Empty();
}

GGA::~GGA()
{
   //Mnemonic.Empty();
   Empty();
}

void GGA::Empty( void )
{
   //UTCTime.Empty();
   Position.Empty();
   GPSQuality                      = 0;
   NumberOfSatellitesInUse         = 0;
   HorizontalDilutionOfPrecision   = 0.0;
   AntennaAltitudeMeters           = 0.0;
   GeoidalSeparationMeters         = 0.0;
   AgeOfDifferentialGPSDataSeconds = 0.0;
   DifferentialReferenceStationID  = 0;
}

BOOL GGA::Parse( const SENTENCE& sentence )
{
   /*
   ** GGA - Global Positioning System Fix Data
   ** Time, Position and fix related data fora GPS receiver.
   **
   **                                                      11
   **        1         2       3 4        5 6 7  8   9  10 |  12 13  14   15
   **        |         |       | |        | | |  |   |   | |   | |   |    |
   ** $--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Universal Time Coordinated (UTC)
   **  2) Latitude
   **  3) N or S (North or South)
   **  4) Longitude
   **  5) E or W (East or West)
   **  6) GPS Quality Indicator,
   **     0 - fix not available,
   **     1 - GPS fix,
   **     2 - Differential GPS fix
   **  7) Number of satellites in view, 00 - 12
   **  8) Horizontal Dilution of precision
   **  9) Antenna Altitude above/below mean-sea-level (geoid) 
   ** 10) Units of antenna altitude, meters
   ** 11) Geoidal separation, the difference between the WGS-84 earth
   **     ellipsoid and mean-sea-level (geoid), "-" means mean-sea-level
   **     below ellipsoid
   ** 12) Units of geoidal separation, meters
   ** 13) Age of differential GPS data, time in seconds since last SC104
   **     type 1 or 9 update, null field when DGPS is not used
   ** 14) Differential reference station ID, 0000-1023
   ** 15) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 15 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   UTCTime                         = sentence.Field( 1 );
   Time                            = sentence.Time( 1 );
   Position.Parse( 2, 3, 4, 5, sentence );
   GPSQuality                      = sentence.Integer( 6 );
   NumberOfSatellitesInUse         = sentence.Integer( 7 );
   HorizontalDilutionOfPrecision   = sentence.Double( 8 );
   AntennaAltitudeMeters           = sentence.Double( 9 );
   GeoidalSeparationMeters         = sentence.Double( 11 );
   AgeOfDifferentialGPSDataSeconds = sentence.Double( 13 );
   DifferentialReferenceStationID  = sentence.Integer( 14 );

   return( TRUE );
}

QString GGA::PlainEnglish( void ) const
{
   QString return_string;

   return_string = "At ";
   return_string += Time.time().toString();
   //return_string += Time.Format( "%I:%M.%S %p" );
   return_string += " UTC, you were at ";
   return_string += Position.PlainEnglish();
   return_string += ", ";

   switch( GPSQuality )
   {
      case 1:

         return_string += "based upon a GPS fix";
         break;

      case 2:

         return_string += "based upon a differential GPS fix";
         break;

      default:

         return_string += "a GPS fix was not available";
         break;
   }

   QString temp_string;
   temp_string = QString::number(NumberOfSatellitesInUse) + " satellites are in use.";
   //temp_string.Format( ", %d satellites are in use.", NumberOfSatellitesInUse );
   return_string += temp_string;

   return( return_string );
}

BOOL GGA::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += UTCTime;
   sentence += Position;
   sentence += GPSQuality;
   sentence += NumberOfSatellitesInUse;
   sentence += HorizontalDilutionOfPrecision;
   sentence += AntennaAltitudeMeters;
   sentence += "M";
   sentence += GeoidalSeparationMeters;
   sentence += "M";
   sentence += AgeOfDifferentialGPSDataSeconds;
   sentence += DifferentialReferenceStationID;

   sentence.Finish();

   return( TRUE );
}

const GGA& GGA::operator = ( const GGA& source )
{
   UTCTime                         = source.UTCTime;
   Time                            = source.Time;
   Position                        = source.Position;
   GPSQuality                      = source.GPSQuality;
   NumberOfSatellitesInUse         = source.NumberOfSatellitesInUse;
   HorizontalDilutionOfPrecision   = source.HorizontalDilutionOfPrecision;
   AntennaAltitudeMeters           = source.AntennaAltitudeMeters;
   GeoidalSeparationMeters         = source.GeoidalSeparationMeters;
   AgeOfDifferentialGPSDataSeconds = source.AgeOfDifferentialGPSDataSeconds;
   DifferentialReferenceStationID  = source.DifferentialReferenceStationID;

   return( *this );
}
