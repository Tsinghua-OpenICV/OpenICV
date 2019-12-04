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
** $Workfile: vpw.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:39p $
*/



VPW::VPW()
{
   Mnemonic = "VPW";
   Empty();
}

VPW::~VPW()
{
   //Mnemonic.Empty();
   Empty();
}

void VPW::Empty( void )
{
   Knots           = 0.0;
   MetersPerSecond = 0.0;
}

BOOL VPW::Parse( const SENTENCE& sentence )
{
   /*
   ** VPW - Speed - Measured Parallel to Wind
   **
   **        1   2 3   4 5
   **        |   | |   | |
   ** $--VPW,x.x,N,x.x,M*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Speed, "-" means downwind
   **  2) N = Knots
   **  3) Speed, "-" means downwind
   **  4) M = Meters per second
   **  5) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 5 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   Knots           = sentence.Double( 1 );
   MetersPerSecond = sentence.Double( 3 );

   return( TRUE );
}

BOOL VPW::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += Knots;
   sentence += "N";
   sentence += MetersPerSecond;
   sentence += "M";

   sentence.Finish();

   return( TRUE );
}

const VPW& VPW::operator = ( const VPW& source )
{
   Knots           = source.Knots;
   MetersPerSecond = source.MetersPerSecond;

   return( *this );
}
