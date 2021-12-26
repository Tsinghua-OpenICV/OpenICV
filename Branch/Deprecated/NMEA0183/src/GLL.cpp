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
** $Workfile: gll.cpp $
** $Revision: 6 $
** $Modtime: 10/12/98 6:39a $
*/



GLL::GLL()
{
   Mnemonic = "GLL";
   Empty();
}

GLL::~GLL()
{
   //Mnemonic.Empty();
   Empty();
}

void GLL::Empty( void )
{
   Position.Empty();
   //UTCTime.Empty();
   IsDataValid = NMEA_Unknown;
}

BOOL GLL::Parse( const SENTENCE& sentence )
{
   /*
   ** GLL - Geographic Position - Latitude/Longitude
   ** Latitude, N/S, Longitude, E/W, UTC, Status
   **
   **        1       2 3        4 5         6 7
   **        |       | |        | |         | |
   ** $--GLL,llll.ll,a,yyyyy.yy,a,hhmmss.ss,A*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Latitude
   **  2) N or S (North or South)
   **  3) Longitude
   **  4) E or W (East or West)
   **  5) Universal Time Coordinated (UTC)
   **  6) Status A - Data Valid, V - Data Invalid
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

   Position.Parse( 1, 2, 3, 4, sentence );
   UTCTime     = sentence.Field( 5 );
   Time        = sentence.Time( 5 );
   IsDataValid = sentence.Boolean( 6 );

   return( TRUE );
}

QString GLL::PlainEnglish( void ) const
{
   QString return_string;

   //return_string.Empty();

   return_string = "At ";
   return_string += Time.time().toString();
   return_string += " UTC, you were at ";
   return_string += Position.PlainEnglish();
   return_string += ".";

   return( return_string );
}

BOOL GLL::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += Position;
   sentence += UTCTime;
   sentence += IsDataValid;

   sentence.Finish();

   return( TRUE );
}

const GLL& GLL::operator = ( const GLL& source )
{
   Position    = source.Position;
   UTCTime     = source.UTCTime;
   Time        = source.Time;
   IsDataValid = source.IsDataValid;

   return( *this );
}
