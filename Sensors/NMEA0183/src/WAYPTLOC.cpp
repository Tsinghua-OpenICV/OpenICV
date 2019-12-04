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
** $Workfile: wayptloc.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:56p $
*/

/*
** This Sentence Not Recommended For New Designs
** A combination of WPL, GLL, ZDA and ZTG is recommended.
*/


WAYPOINT_LOCATION::WAYPOINT_LOCATION()
{
   //Mnemonic.Empty();
   Empty();
}

WAYPOINT_LOCATION::~WAYPOINT_LOCATION()
{
   //Mnemonic.Empty();
   Empty();
}

void WAYPOINT_LOCATION::Empty( void )
{
   Position.Empty();
   //UTCTime.Empty();
}

BOOL WAYPOINT_LOCATION::Parse( const SENTENCE& sentence )
{
   /*
   ** xxx - Waypoint location
   **
   **        1         2       3 4        5 6    7
   **        |         |       | |        | |    |
   ** $--xxx,hhmmss.ss,llll.ll,a,yyyyy.yy,a,c--c*hh<CR><LF>
   **
   ** Field Number: 
   **  1) UTC Time
   **  2) Latitude
   **  3) N or S (North or South)
   **  4) Longitude
   **  5) E or W (East or West)
   **  6) Waypoint name
   **  7) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 7 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   UTCTime  = sentence.Field( 1 );
   Time     = sentence.Time( 1 );
   Position.Parse( 2, 3, 4, 5, sentence );
   Waypoint = sentence.Field( 6 );

   return( TRUE );
}

QString WAYPOINT_LOCATION::PlainEnglish( void ) const
{
   QString return_string;

   //return_string.Empty();

   return_string = "At ";
   //return_string += Time.Format( "%I:%M.%S %p" );
   return_string += Time.time().toString();
   return_string += ", you were at waypoint ";
   return_string += Waypoint;
   return_string += " located at ";
   return_string += Position.PlainEnglish();
   return_string += ".";

   return( return_string );
}

BOOL WAYPOINT_LOCATION::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += UTCTime;
   sentence += Position;
   sentence += Waypoint;

   sentence.Finish();

   return( TRUE );
}

const WAYPOINT_LOCATION& WAYPOINT_LOCATION::operator = ( const WAYPOINT_LOCATION& source )
{
   UTCTime  = source.UTCTime;
   Time     = source.Time;
   Position = source.Position;
   Waypoint = source.Waypoint;

   return( *this );
}
