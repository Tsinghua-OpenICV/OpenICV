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
** $Workfile: ratiopls.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:43p $
*/


RATIO_AND_PULSE::RATIO_AND_PULSE()
{
   Empty();
}

RATIO_AND_PULSE::~RATIO_AND_PULSE()
{
   Empty();
}

void RATIO_AND_PULSE::Empty( void )
{
   SignalToNoiseRatio = 0;
   PulseShapeECD      = 0;
}

void RATIO_AND_PULSE::Parse( int first_field_number, const SENTENCE& sentence )
{
   SignalToNoiseRatio = sentence.Integer( first_field_number );
   PulseShapeECD      = sentence.Integer( first_field_number + 1 );
}

void RATIO_AND_PULSE::Write( SENTENCE& sentence )
{
   sentence += SignalToNoiseRatio;
   sentence += PulseShapeECD;
}

const RATIO_AND_PULSE& RATIO_AND_PULSE::operator = ( const RATIO_AND_PULSE& source )
{
   SignalToNoiseRatio = source.SignalToNoiseRatio;
   PulseShapeECD      = source.PulseShapeECD;

   return( *this );
}
