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
** $Workfile: bww.cpp $
** $Revision: 5 $
** $Modtime: 10/10/98 2:45p $
*/



BWW::BWW()
{
   Mnemonic = "BWW";
   Empty();
}

BWW::~BWW()
{
   //Mnemonic.Empty();
   Empty();
}

void BWW::Empty( void )
{
   BearingTrue     = 0.0;
   BearingMagnetic = 0.0;
   //To.Empty();
   //From.Empty();
}

BOOL BWW::Parse( const SENTENCE& sentence )
{
   /*
   ** BWW - Bearing - Waypoint to Waypoint
   **
   **        1   2 3   4 5    6    7
   **        |   | |   | |    |    |
   ** $--BWW,x.x,T,x.x,M,c--c,c--c*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Bearing Degrees, TRUE
   **  2) T = True
   **  3) Bearing Degrees, Magnetic
   **  4) M = Magnetic
   **  5) TO Waypoint
   **  6) FROM Waypoint
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

   BearingTrue     = sentence.Double( 1 );
   BearingMagnetic = sentence.Double( 3 );
   To              = sentence.Field( 5 );
   From            = sentence.Field( 6 );

   return( TRUE );
}

BOOL BWW::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += BearingTrue;
   sentence += "T";
   sentence += BearingMagnetic;
   sentence += "M";
   sentence += To;
   sentence += From;

   sentence.Finish();

   return( TRUE );
}

const BWW& BWW::operator = ( const BWW& source )
{
   BearingTrue     = source.BearingTrue;
   BearingMagnetic = source.BearingMagnetic;
   To              = source.To;
   From            = source.From;

   return( *this );
}
