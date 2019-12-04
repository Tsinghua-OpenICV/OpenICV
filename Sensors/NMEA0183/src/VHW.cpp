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
** $Workfile: vhw.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:38p $
*/



VHW::VHW()
{
   Mnemonic = "VHW";
   Empty();
}

VHW::~VHW()
{
   //Mnemonic.Empty();
   Empty();
}

void VHW::Empty( void )
{
   DegreesTrue       = 0.0;
   DegreesMagnetic   = 0.0;
   Knots             = 0.0;
   KilometersPerHour = 0.0;
}

BOOL VHW::Parse( const SENTENCE& sentence )
{
   /*
   ** VHW - Water speed and heading
   **
   **        1   2 3   4 5   6 7   8 9
   **        |   | |   | |   | |   | |
   ** $--VHW,x.x,T,x.x,M,x.x,N,x.x,K*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Degress True
   **  2) T = True
   **  3) Degrees Magnetic
   **  4) M = Magnetic
   **  5) Knots (speed of vessel relative to the water)
   **  6) N = Knots
   **  7) Kilometers (speed of vessel relative to the water)
   **  8) K = Kilometers
   **  9) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 9 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   DegreesTrue       = sentence.Double( 1 );
   DegreesMagnetic   = sentence.Double( 3 );
   Knots             = sentence.Double( 5 );
   KilometersPerHour = sentence.Double( 7 );

   return( TRUE );
}

BOOL VHW::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += DegreesTrue;
   sentence += "T";
   sentence += DegreesMagnetic;
   sentence += "M";
   sentence += Knots;
   sentence += "N";
   sentence += KilometersPerHour;
   sentence += "K";

   sentence.Finish();

   return( TRUE );
}

const VHW& VHW::operator = ( const VHW& source )
{
   DegreesTrue       = source.DegreesTrue;
   DegreesMagnetic   = source.DegreesMagnetic;
   Knots             = source.Knots;
   KilometersPerHour = source.KilometersPerHour;

   return( *this );
}
