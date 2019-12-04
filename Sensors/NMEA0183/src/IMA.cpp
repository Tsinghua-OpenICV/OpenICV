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
** $Workfile: ima.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:44p $
*/

/*
** This Sentence Not Recommended For New Designs
** There is no recommended replacement.
*/



IMA::IMA()
{
   Mnemonic = "IMA";
   Empty();
}

IMA::~IMA()
{
   //Mnemonic.Empty();
   Empty();
}

void IMA::Empty( void )
{
   //VesselName.Empty();
   //Callsign.Empty();
   HeadingDegreesTrue     = 0.0;
   HeadingDegreesMagnetic = 0.0;
   SpeedKnots             = 0.0;
}

BOOL IMA::Parse( const SENTENCE& sentence )
{
   /*
   ** IMA - Vessel Identification
   **                                                              11    13
   **        1            2       3       4 5        6 7   8 9   10|   12|
   **        |            |       |       | |        | |   | |   | |   | |
   ** $--IMA,aaaaaaaaaaaa,aaaxxxx,llll.ll,a,yyyyy.yy,a,x.x,T,x.x,M,x.x,N*hh<CR><LF>
   **
   **  1) Twelve character vessel name
   **  2) Radio Call Sign
   **  3) Latitude
   **  4) North/South
   **  5) Longitude
   **  6) East/West
   **  7) Heading, degrees true
   **  8) T = True
   **  9) Heading, degrees magnetic
   ** 10) M = Magnetic
   ** 11) Speed
   ** 12) N = Knots
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

   VesselName             = sentence.Field( 1 );
   Callsign               = sentence.Field( 2 );
   Position.Parse( 3, 4, 5, 6, sentence );
   HeadingDegreesTrue     = sentence.Double( 7 );
   HeadingDegreesMagnetic = sentence.Double( 9 );
   SpeedKnots             = sentence.Double( 11 );

   return( TRUE );
}

BOOL IMA::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += VesselName;
   sentence += Callsign;
   sentence += Position;
   sentence += HeadingDegreesTrue;
   sentence += "T";
   sentence += HeadingDegreesMagnetic;
   sentence += "M";
   sentence += SpeedKnots;
   sentence += "N";

   sentence.Finish();

   return( TRUE );
}

const IMA& IMA::operator = ( const IMA& source )
{
   VesselName             = source.VesselName;
   Callsign               = source.Callsign;
   Position               = source.Position;
   HeadingDegreesTrue     = source.HeadingDegreesTrue;
   HeadingDegreesMagnetic = source.HeadingDegreesMagnetic;
   SpeedKnots             = source.SpeedKnots;

   return( *this );
}
