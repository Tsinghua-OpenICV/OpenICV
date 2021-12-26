#include "nmea0183.h"
#pragma hdrstop

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: trf.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:40p $
*/



TRF::TRF()
{
   Mnemonic = "TRF";
   Empty();
}

TRF::~TRF()
{
   //Mnemonic.Empty();
   Empty();
}

void TRF::Empty( void )
{
   //UTCTime.Empty();
   //Date.Empty();
   Position.Empty();
   ElevationAngle              = 0.0;
   NumberOfIterations          = 0.0;
   NumberOfDopplerIntervals    = 0.0;
   UpdateDistanceNauticalMiles = 0.0;
   SatelliteID                 = 0;
   IsDataValid                 = NMEA_Unknown;
}

BOOL TRF::Parse( const SENTENCE& sentence )
{
   /*
   ** TRF - TRANSIT Fix Data
   **                                                                    13
   **        1         2      3       4 5        6 7   8   9   10  11  12|
   **        |         |      |       | |        | |   |   |   |   |   | |
   ** $--TRF,hhmmss.ss,xxxxxx,llll.ll,a,yyyyy.yy,a,x.x,x.x,x.x,x.x,xxx,A*hh<CR><LF>
   **
   ** Field Number: 
   **  1) UTC Time
   **  2) Date, ddmmyy
   **  3) Latitude
   **  4) N or S
   **  5) Longitude
   **  6) E or W
   **  7) Elevation Angle
   **  8) Number of iterations
   **  9) Number of Doppler intervals
   ** 10) Update distance, nautical miles
   ** 11) Satellite ID
   ** 12) Data Validity
   ** 13) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 13 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   UTCTime                     = sentence.Field( 1 );
   Time                        = sentence.Time( 1 );
   Date                        = sentence.Field( 2 );
   Position.Parse( 3, 4, 5, 6, sentence );
   ElevationAngle              = sentence.Double( 7 );
   NumberOfIterations          = sentence.Double( 8 );
   NumberOfDopplerIntervals    = sentence.Double( 9 );
   UpdateDistanceNauticalMiles = sentence.Double( 10 );
   SatelliteID                 = sentence.Integer( 11 );
   IsDataValid                 = sentence.Boolean( 12 );

   return( TRUE );
}

BOOL TRF::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += UTCTime;
   sentence += Date;
   sentence += Position;
   sentence += ElevationAngle;
   sentence += NumberOfIterations;
   sentence += NumberOfDopplerIntervals;
   sentence += UpdateDistanceNauticalMiles;
   sentence += SatelliteID;
   sentence += IsDataValid;

   sentence.Finish();

   return( TRUE );
}

const TRF& TRF::operator = ( const TRF& source )
{
   UTCTime                     = source.UTCTime;
   Time                        = source.Time;
   Date                        = source.Date;
   Position                    = source.Position;
   ElevationAngle              = source.ElevationAngle;
   NumberOfIterations          = source.NumberOfIterations;
   NumberOfDopplerIntervals    = source.NumberOfDopplerIntervals;
   UpdateDistanceNauticalMiles = source.UpdateDistanceNauticalMiles;
   SatelliteID                 = source.SatelliteID;
   IsDataValid                 = source.IsDataValid;

  return( *this );
}
