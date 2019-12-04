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
** $Workfile: omegapar.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:42p $
*/


OMEGA_PAIR::OMEGA_PAIR()
{
   Empty();
}

OMEGA_PAIR::~OMEGA_PAIR()
{
   Empty();
}

void OMEGA_PAIR::Empty( void )
{
   //Name.Empty();
   LaneNumber      = 0;
   CentilaneNumber = 0;
}

void OMEGA_PAIR::Parse( int first_field_number, const SENTENCE& sentence )
{
   Name            = sentence.Field( first_field_number );
   LaneNumber      = sentence.Integer( first_field_number + 1 );
   CentilaneNumber = sentence.Integer( first_field_number + 2 );
}

void OMEGA_PAIR::Write( SENTENCE& sentence )
{
   sentence += Name;
   sentence += LaneNumber;
   sentence += CentilaneNumber;
}

const OMEGA_PAIR& OMEGA_PAIR::operator = ( const OMEGA_PAIR& source )
{
   Name            = source.Name;
   LaneNumber      = source.LaneNumber;
   CentilaneNumber = source.CentilaneNumber;

   return( *this );
}
