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
** $Workfile: satdat.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:43p $
*/


SATELLITE_DATA::SATELLITE_DATA()
{
   Empty();
}

SATELLITE_DATA::~SATELLITE_DATA()
{
   Empty();
}

void SATELLITE_DATA::Empty( void )
{
   SatelliteNumber    = 0;
   ElevationDegrees   = 0;
   AzimuthDegreesTrue = 0;
   SignalToNoiseRatio = (-1);
}

void SATELLITE_DATA::Parse( int first_field_number, const SENTENCE& sentence )
{
   SatelliteNumber    = sentence.Integer( first_field_number );
   ElevationDegrees   = sentence.Integer( first_field_number + 1 );
   AzimuthDegreesTrue = sentence.Integer( first_field_number + 2 );
   
   QString field_data = sentence.Field( first_field_number + 3 );

   if ( field_data == "" )
   {
      SignalToNoiseRatio = (-1);
   }
   else
   {
      SignalToNoiseRatio = sentence.Integer( first_field_number + 3 );
   }
     
}

void SATELLITE_DATA::Write( SENTENCE& sentence )
{
   sentence += SatelliteNumber;
   sentence += ElevationDegrees;
   sentence += AzimuthDegreesTrue;

   if ( SignalToNoiseRatio == (-1) )
   {
      sentence += "";
   }
   else
   {
      sentence += SignalToNoiseRatio;
   }
}

const SATELLITE_DATA& SATELLITE_DATA::operator = ( const SATELLITE_DATA& source )
{
   SatelliteNumber    = source.SatelliteNumber;
   ElevationDegrees   = source.ElevationDegrees;
   AzimuthDegreesTrue = source.AzimuthDegreesTrue;
   SignalToNoiseRatio = source.SignalToNoiseRatio;

   return( *this );
}
