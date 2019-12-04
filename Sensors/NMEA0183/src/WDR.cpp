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
** $Workfile: wdr.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:39p $
*/

/*
** This Sentence Not Recommended For New Designs
** BWC is recommended.
*/



WDR::WDR()
{
   Mnemonic = "WDR";
   Empty();
}

WDR::~WDR()
{
   //Mnemonic.Empty();
   Empty();
}

void WDR::Empty( void )
{
   NauticalMiles = 0;
   //To.Empty();
}

BOOL WDR::Parse( const SENTENCE& sentence )
{
   /*
   ** WDR - Distance to Waypoint, Rhumb Line
   **
   **        1   2 3    4
   **        |   | |    |
   ** $--WDR,x.x,N,c--c*hh<CR><LF>
   **
   ** 1) Distance to waypoint
   ** 2) N = Nautical Miles
   ** 3) Waypoint ID (To)
   ** 4) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 4 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   }

   NauticalMiles = sentence.Double( 1 );
   To            = sentence.Field( 3 );

   return( TRUE );
}

BOOL WDR::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += NauticalMiles;
   sentence += "N";
   sentence += To;

   sentence.Finish();

   return( TRUE );
}

const WDR& WDR::operator = ( const WDR& source )
{
   NauticalMiles = source.NauticalMiles;
   To            = source.To;

   return( *this );
}
