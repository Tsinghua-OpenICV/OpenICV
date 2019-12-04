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
** $Workfile: wcv.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:39p $
*/


WCV::WCV()
{
   Mnemonic = "WCV";
   Empty();
}

WCV::~WCV()
{
   //Mnemonic.Empty();
   Empty();
}

void WCV::Empty( void )
{
   Velocity = 0.0;
   //To.Empty();
}

BOOL WCV::Parse( const SENTENCE& sentence )
{
   /*
   ** WCV - Waypoint Closure Velocity
   **
   **        1   2 3    4
   **        |   | |    |
   ** $--WCV,x.x,N,c--c*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Velocity
   **  2) N = knots
   **  3) Waypoint ID
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

   Velocity = sentence.Double( 1 );
   To       = sentence.Field( 3 );

   return( TRUE );
}

BOOL WCV::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += Velocity;
   sentence += "N";
   sentence += To;

   sentence.Finish();
   return( TRUE );
}

const WCV& WCV::operator = ( const WCV& source )
{
   Velocity = source.Velocity;
   To       = source.To;

   return( *this );
}
