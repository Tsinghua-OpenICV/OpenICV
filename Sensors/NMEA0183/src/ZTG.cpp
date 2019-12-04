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
** $Workfile: ztg.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:38p $
*/


ZTG::ZTG()
{
   Mnemonic = "ZTG";
   Empty();
}

ZTG::~ZTG()
{
   //Mnemonic.Empty();
   Empty();
}

void ZTG::Empty( void )
{
   //UTCTime.Empty();
   //TimeRemaining.Empty();
   //To.Empty();
}

BOOL ZTG::Parse( const SENTENCE& sentence )
{
   /*
   ** ZTG - UTC & Time to Destination Waypoint
   **
   **        1         2         3    4
   **        |         |         |    |
   ** $--ZTG,hhmmss.ss,hhmmss.ss,c--c*hh<CR><LF>
   **
   ** Fields:
   **  1) Universal Time Coordinated (UTC)
   **  2) Time Remaining
   **  3) Destination Waypoint ID
   **  4) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 4 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   UTCTime       = sentence.Field( 1 );
   Time          = sentence.Time( 1 );
   TimeRemaining = sentence.Field( 2 );
   To            = sentence.Field( 3 );

   return( TRUE );
}

BOOL ZTG::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += UTCTime;
   sentence += TimeRemaining;
   sentence += To;

   sentence.Finish();

   return( TRUE );
}

const ZTG& ZTG::operator = ( const ZTG& source )
{
   UTCTime       = source.UTCTime;
   Time          = source.Time;
   TimeRemaining = source.TimeRemaining;
   To            = source.To;

   return( *this );
}
