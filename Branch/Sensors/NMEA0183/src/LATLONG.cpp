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
** $Workfile: latlong.cpp $
** $Revision: 5 $
** $Modtime: 10/13/98 6:39a $
*/

#ifdef _MSC_VER
#   pragma warning(disable:4996)
#endif // _MSC_VER

LATLONG::LATLONG()
{
   Empty();
}

LATLONG::~LATLONG()
{
   Empty();
}

void LATLONG::Empty( void )
{
   Latitude.Empty();
   Longitude.Empty();
}

BOOL LATLONG::Parse( int LatitudePositionFieldNumber, int NorthingFieldNumber, int LongitudePositionFieldNumber, int EastingFieldNumber, const SENTENCE& LineToParse )
{
   Latitude.Parse(  LatitudePositionFieldNumber, NorthingFieldNumber, LineToParse );
   Longitude.Parse( LongitudePositionFieldNumber, EastingFieldNumber, LineToParse );

   if ( Latitude.IsDataValid() && Longitude.IsDataValid() )
   {
      return( TRUE );
   }
   else
   {
      return( FALSE );
   }
}

QString LATLONG::PlainEnglish( void ) const
{
   QString return_string;

   //return_string.Format( "Latitude %d %8.5lf", Latitude.GetWholeDegrees(), Latitude.GetDecimalMinutes() );
   return_string = "Latitude " + QString::number(Latitude.GetWholeDegrees()) + " " + QString::number(Latitude.GetDecimalMinutes());

   if ( Latitude.Northing == North )
   {
      return_string += " North, Longitude ";
   }
   else
   {
      return_string += " South, Longitude ";
   }

   char temp_string[ 80 ];

   ::sprintf( temp_string, "%d %8.5lf", Longitude.GetWholeDegrees(), Longitude.GetDecimalMinutes() );

   return_string += temp_string;

   if ( Longitude.Easting == East )
   {
      return_string += " East";
   }
   else
   {
      return_string += " West";
   }

   return( return_string );
}

void LATLONG::Write( SENTENCE& sentence )
{
   Latitude.Write( sentence );
   Longitude.Write( sentence );
}

const LATLONG& LATLONG::operator = ( const LATLONG& source )
{
   Latitude  = source.Latitude;
   Longitude = source.Longitude;

   return( *this );
}
