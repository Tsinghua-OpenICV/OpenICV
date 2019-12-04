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
** $Workfile: freqmode.cpp $
** $Revision: 5 $
** $Modtime: 10/10/98 2:46p $
*/


FREQUENCY_AND_MODE::FREQUENCY_AND_MODE()
{
   Empty();
}

FREQUENCY_AND_MODE::~FREQUENCY_AND_MODE()
{
   Empty();
}

void FREQUENCY_AND_MODE::Empty( void )
{
   Frequency = 0.0;
   Mode      = CommunicationsModeUnknown;
}

void FREQUENCY_AND_MODE::Parse( int first_field_number, const SENTENCE& sentence )
{
   Frequency = sentence.Double( first_field_number );
   Mode      = sentence.CommunicationsMode( first_field_number + 1 );
}

void FREQUENCY_AND_MODE::Write( SENTENCE& sentence )
{
   sentence += Frequency;
   sentence += Mode;
}

const FREQUENCY_AND_MODE& FREQUENCY_AND_MODE::operator = ( const FREQUENCY_AND_MODE& source )
{
   Frequency = source.Frequency;
   Mode      = source.Mode;

   return( *this );
}
