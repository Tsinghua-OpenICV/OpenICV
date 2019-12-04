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
** $Workfile: zfo.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:40p $
*/



ZFO::ZFO()
{
   Mnemonic = "ZFO";
   Empty();
}

ZFO::~ZFO()
{
   //Mnemonic.Empty();
   Empty();
}

void ZFO::Empty( void )
{
   //UTCTime.Empty();
   //ElapsedTime.Empty();
   //From.Empty();
}

BOOL ZFO::Parse( const SENTENCE& sentence )
{
   /*
   ** ZFO - UTC & Time from origin Waypoint
   **
   **        1         2         3    4
   **        |         |         |    |
   ** $--ZFO,hhmmss.ss,hhmmss.ss,c--c*hh<CR><LF>
   **
   ** Fields:
   **  1) Universal Time Coordinated (UTC)
   **  2) Elapsed Time
   **  3) Origin Waypoint ID
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

   UTCTime     = sentence.Field( 1 );
   Time        = sentence.Time( 1 );
   ElapsedTime = sentence.Field( 2 );
   From        = sentence.Field( 3 );

   return( TRUE );
}

BOOL ZFO::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += UTCTime;
   sentence += ElapsedTime;
   sentence += From;

   sentence.Finish();

   return( TRUE );
}

const ZFO& ZFO::operator = ( const ZFO& source )
{
   UTCTime     = source.UTCTime;
   Time        = source.Time;
   ElapsedTime = source.ElapsedTime;
   From        = source.From;

   return( *this );
}
