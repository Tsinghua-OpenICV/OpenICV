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
** $Workfile: lorantd.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:44p $
*/


LORAN_TIME_DIFFERENCE::LORAN_TIME_DIFFERENCE()
{
   Empty();
}

LORAN_TIME_DIFFERENCE::~LORAN_TIME_DIFFERENCE()
{
   Empty();
}

void LORAN_TIME_DIFFERENCE::Empty( void )
{
   Microseconds = 0.0;
   SignalStatus = LoranUnknown;
}

void LORAN_TIME_DIFFERENCE::Parse( int first_field_number, const SENTENCE& sentence )
{
   Microseconds = sentence.Double( first_field_number );

   QString field_data = sentence.Field( first_field_number + 1 );

   if ( field_data == "B" )
   {
      SignalStatus = LoranBlinkWarning;   
   }
   else if ( field_data == "C" )
   {
      SignalStatus = LoranCycleWarning;
   }
   else if ( field_data == "C" )
   {
      SignalStatus = LoranSignalToNoiseRatioWarning;
   }
   else if ( field_data == "A" )
   {
      SignalStatus = LoranValid;
   }
   else
   {
      SignalStatus = LoranUnknown;
   }
}

void LORAN_TIME_DIFFERENCE::Write( SENTENCE& sentence )
{
   sentence += Microseconds;

   switch( SignalStatus )
   {
      case LoranValid:
      
         sentence += "A";
         break; 

      case LoranBlinkWarning:
      
         sentence += "B";
         break; 

      case LoranCycleWarning:
      
         sentence += "C";
         break; 

      case LoranSignalToNoiseRatioWarning:
      
         sentence += "S";
         break; 

      default:

         sentence += "";
   }
}

const LORAN_TIME_DIFFERENCE& LORAN_TIME_DIFFERENCE::operator = ( const LORAN_TIME_DIFFERENCE& source )
{
   Microseconds = source.Microseconds;
   SignalStatus = source.SignalStatus;

   return( *this );
}
