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
** $Workfile: radardat.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:42p $
*/


RADAR_DATA::RADAR_DATA()
{
   Empty();
}

RADAR_DATA::~RADAR_DATA()
{
   Empty();
}

void RADAR_DATA::Empty( void )
{
   OriginRangeFromOwnShip       = 0.0;
   OriginBearingDegreesFromZero = 0.0;
   VariableRangeMarkerRange     = 0.0;
   BearingLineDegreesFromZero   = 0.0;
}

void RADAR_DATA::Parse( int first_field_number, const SENTENCE& sentence )
{
   OriginRangeFromOwnShip       = sentence.Double( first_field_number );
   OriginBearingDegreesFromZero = sentence.Double( first_field_number + 1 );
   VariableRangeMarkerRange     = sentence.Double( first_field_number + 2 );
   BearingLineDegreesFromZero   = sentence.Double( first_field_number + 3 );
}

void RADAR_DATA::Write( SENTENCE& sentence )
{
   sentence += OriginRangeFromOwnShip;
   sentence += OriginBearingDegreesFromZero;
   sentence += VariableRangeMarkerRange;
   sentence += BearingLineDegreesFromZero;
}

const RADAR_DATA& RADAR_DATA::operator = ( const RADAR_DATA& source )
{
   OriginRangeFromOwnShip       = source.OriginRangeFromOwnShip;
   OriginBearingDegreesFromZero = source.OriginBearingDegreesFromZero;
   VariableRangeMarkerRange     = source.VariableRangeMarkerRange;
   BearingLineDegreesFromZero   = source.BearingLineDegreesFromZero;

   return( *this );
}
