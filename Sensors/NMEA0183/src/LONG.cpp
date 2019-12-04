#include "nmea0183.h"
#pragma hdrstop

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1997, Samuel R. Blackburn
**
** $Workfile: long.cpp $
** $Revision: 6 $
** $Modtime: 10/12/98 6:50a $
*/

#ifdef _MSC_VER
#   pragma warning(disable:4996)
#endif // _MSC_VER

LONGITUDE::LONGITUDE()
{
   Empty();
}

LONGITUDE::~LONGITUDE()
{
   Empty();
}

void LONGITUDE::Empty( void )
{
   Longitude = 0.0;
   Easting   = EW_Unknown;
}

BOOL LONGITUDE::IsDataValid( void )
{
   if ( Easting != East && Easting != West )
   {
      return( FALSE );
   }

   return( TRUE );
}

void LONGITUDE::Parse( int position_field_number, int east_or_west_field_number, const SENTENCE& sentence )
{
   // Thanks go to Eric Parsonage (ericpa@mpx.com.au) for finding a nasty
   // little bug that used to live here.

   double position = 0.0;

   position = sentence.Double( position_field_number );

   QString east_or_west;

   east_or_west = sentence.Field( east_or_west_field_number );

   Set( position, east_or_west.toLatin1() );
}

void LONGITUDE::Set( double position, const char *east_or_west )
{
   Longitude  = position;
   Coordinate = position;

   if ( east_or_west[ 0 ] == 'E' )
   {
      Easting = East;
   }
   else if ( east_or_west[ 0 ] == 'W' )
   {
      Easting = West;
   }
   else
   {
      Easting = EW_Unknown;
   }
}

void LONGITUDE::Write( SENTENCE& sentence )
{
   char temp_string[ 80 ];

   ::sprintf( temp_string, "%08.2f", Longitude );
   sentence += temp_string;
   
   if ( Easting == East )
   {
      sentence += "E";
   }
   else if ( Easting == West )
   {
      sentence += "W";
   }
   else
   {
      /*
      ** Thanks to Jan-Erik Eriksson (Jan-Erik.Eriksson@st.se) for
      ** finding and fixing a bug here
      */

      sentence += "";
   }
}

const LONGITUDE& LONGITUDE::operator = ( const LONGITUDE& source )
{
   Longitude = source.Longitude;
   Easting   = source.Easting;

   return( *this );
}
