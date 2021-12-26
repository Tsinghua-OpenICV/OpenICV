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
** $Workfile: deccalop.cpp $
** $Revision: 5 $
** $Modtime: 10/10/98 2:46p $
*/


LINE_OF_POSITION::LINE_OF_POSITION()
{
   Empty();
}

LINE_OF_POSITION::~LINE_OF_POSITION()
{
   Empty();
}

void LINE_OF_POSITION::Empty( void )
{
   //ZoneID.Empty();
   LineOfPosition = 0.0;
   MasterLine     = NMEA_Unknown;
}

void LINE_OF_POSITION::Parse( int first_field_number, const SENTENCE& sentence )
{
   ZoneID         = sentence.Field(   first_field_number );
   LineOfPosition = sentence.Double(  first_field_number + 1 );
   MasterLine     = sentence.Boolean( first_field_number + 2 );
}

void LINE_OF_POSITION::Write( SENTENCE& sentence )
{
   sentence += ZoneID;
   sentence += LineOfPosition;
   sentence += MasterLine;
}

const LINE_OF_POSITION& LINE_OF_POSITION::operator = ( const LINE_OF_POSITION& source )
{
   ZoneID         = source.ZoneID;
   LineOfPosition = source.LineOfPosition;
   MasterLine     = source.MasterLine;

   return( *this );
}
