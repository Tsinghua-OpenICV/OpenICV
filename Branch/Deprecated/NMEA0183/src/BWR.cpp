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
** $Workfile: bwr.cpp $
** $Revision: 5 $
** $Modtime: 10/10/98 2:41p $
*/



BWR::BWR()
{
   Mnemonic = "BWR";
   Empty();
}

BWR::~BWR()
{
   //Mnemonic.Empty();
   Empty();
}

void BWR::Empty( void )
{
   //UTCTime.Empty();
   BearingTrue     = 0.0;
   BearingMagnetic = 0.0;
   NauticalMiles   = 0.0;
   //To.Empty();
}

BOOL BWR::Parse( const SENTENCE& sentence )
{
   /*
   ** BWR - Bearing and Distance to Waypoint - Rhumb Line
   ** Latitude, N/S, Longitude, E/W, UTC, Status
   **                                                       11
   **        1         2       3 4        5 6   7 8   9 10  | 12   13
   **        |         |       | |        | |   | |   | |   | |    |
   ** $--BWR,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x.x,T,x.x,M,x.x,N,c--c*hh<CR><LF>
   **
   **  1) UTCTime
   **  2) Waypoint Latitude
   **  3) N = North, S = South
   **  4) Waypoint Longitude
   **  5) E = East, W = West
   **  6) Bearing, True
   **  7) T = True
   **  8) Bearing, Magnetic
   **  9) M = Magnetic
   ** 10) Nautical Miles
   ** 11) N = Nautical Miles
   ** 12) Waypoint ID
   ** 13) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 13 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   UTCTime         = sentence.Field( 1 );
   Time            = sentence.Time( 1 );
   Position.Parse( 2, 3, 4, 5, sentence );
   BearingTrue     = sentence.Double( 6 );
   BearingMagnetic = sentence.Double( 8 );
   NauticalMiles   = sentence.Double( 10 );
   To              = sentence.Field( 12 );

   return( TRUE );
}

BOOL BWR::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += UTCTime;
   sentence += Position;
   sentence += BearingTrue;
   sentence += "T";
   sentence += BearingMagnetic;
   sentence += "M";
   sentence += NauticalMiles;
   sentence += "N";
   sentence += To;

   sentence.Finish();

   return( TRUE );
}

const BWR& BWR::operator = ( const BWR& source )
{
   UTCTime         = source.UTCTime;
   Time            = source.Time;
   Position        = source.Position;
   BearingTrue     = source.BearingTrue;
   BearingMagnetic = source.BearingMagnetic;
   NauticalMiles   = source.NauticalMiles;
   To              = source.To;

   return( *this );
}
