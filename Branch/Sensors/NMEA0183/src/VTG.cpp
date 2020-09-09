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
** $Workfile: vtg.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:39p $
*/



VTG::VTG()
{
   Mnemonic = "VTG";
   Empty();
}

VTG::~VTG()
{
   //Mnemonic.Empty();
   Empty();
}

void VTG::Empty( void )
{
   TrackDegreesTrue       = 0.0;
   TrackDegreesMagnetic   = 0.0;
   SpeedKnots             = 0.0;
   SpeedKilometersPerHour = 0.0;
}

BOOL VTG::Parse( const SENTENCE& sentence )
{
   /*
   ** VTG - Track made good and Ground speed
   **
   **        1   2 3   4 5	 6 7   8 9
   **        |   | |   | |	 | |   | |
   ** $--VTG,x.x,T,x.x,M,x.x,N,x.x,K*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Track Degrees
   **  2) T = True
   **  3) Track Degrees
   **  4) M = Magnetic
   **  5) Speed Knots
   **  6) N = Knots
   **  7) Speed Kilometers Per Hour
   **  8) K = Kilometers Per Hour
   **  9) Checksum
   */

   /*
   ** First we check the checksum...
   ** the param '0' force automatic searching of checksum after '*' delimiter
   */

   if ( sentence.IsChecksumBad( 0 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   TrackDegreesTrue       = sentence.Double( 1 );
   TrackDegreesMagnetic   = sentence.Double( 3 );
   SpeedKnots             = sentence.Double( 5 );
   SpeedKilometersPerHour = sentence.Double( 7 );

   return( TRUE );
}

BOOL VTG::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += TrackDegreesTrue;
   sentence += "T";
   sentence += TrackDegreesMagnetic;
   sentence += "M";
   sentence += SpeedKnots;
   sentence += "N";
   sentence += SpeedKilometersPerHour;
   sentence += "K";

   sentence.Finish();

   return( TRUE );
}

const VTG& VTG::operator = ( const VTG& source )
{
   TrackDegreesTrue       = source.TrackDegreesTrue;
   TrackDegreesMagnetic   = source.TrackDegreesMagnetic;
   SpeedKnots             = source.SpeedKnots;
   SpeedKilometersPerHour = source.SpeedKilometersPerHour;

   return( *this );
}
