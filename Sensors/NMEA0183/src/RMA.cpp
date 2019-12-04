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
** $Workfile: rma.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:43p $
*/



RMA::RMA()
{
   Mnemonic = "RMA";
   Empty();
}

RMA::~RMA()
{
   //Mnemonic.Empty();
   Empty();
}

void RMA::Empty( void )
{
   IsDataValid                = NMEA_Unknown;
   TimeDifferenceA            = 0.0;
   TimeDifferenceB            = 0.0;
   SpeedOverGroundKnots       = 0.0;
   TrackMadeGoodDegreesTrue   = 0.0;
   MagneticVariation          = 0.0;
   MagneticVariationDirection = EW_Unknown;
}

BOOL RMA::Parse( const SENTENCE& sentence )
{
   /*
   ** RMA - Recommended Minimum Navigation Information
   **                                                    12
   **        1 2       3 4        5 6   7   8   9   10  11|
   **        | |       | |        | |   |   |   |   |   | |
   ** $--RMA,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,x.x,x.x,x.x,a*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Blink Warning
   **  2) Latitude
   **  3) N or S
   **  4) Longitude
   **  5) E or W
   **  6) Time Difference A, uS
   **  7) Time Difference B, uS
   **  8) Speed Over Ground, Knots
   **  9) Track Made Good, degrees true
   ** 10) Magnetic Variation, degrees
   ** 11) E or W
   ** 12) Checksum
   */

   /*
   ** First we check the checksum...
   */

   NMEA0183_BOOLEAN check = sentence.IsChecksumBad( 12 );

   if ( check == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   }
   
   if ( check == NMEA_Unknown )
   {
      SetErrorMessage( "Missing Checksum" );
      return( FALSE );
   } 

   IsDataValid                = sentence.Boolean( 1 );
   Position.Parse( 2, 3, 4, 5, sentence );
   TimeDifferenceA            = sentence.Double( 6 );
   TimeDifferenceB            = sentence.Double( 7 );
   SpeedOverGroundKnots       = sentence.Double( 8 );
   TrackMadeGoodDegreesTrue   = sentence.Double( 9 );
   MagneticVariation          = sentence.Double( 10 );
   MagneticVariationDirection = sentence.EastOrWest( 11 );

   return( TRUE );
}

BOOL RMA::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += IsDataValid;
   sentence += Position;
   sentence += TimeDifferenceA;
   sentence += TimeDifferenceB;
   sentence += SpeedOverGroundKnots;
   sentence += TrackMadeGoodDegreesTrue;
   sentence += MagneticVariation;
   sentence += MagneticVariationDirection;

   sentence.Finish();

   return( TRUE );
}

const RMA& RMA::operator = ( const RMA& source )
{
   IsDataValid                = source.IsDataValid;
   Position                   = source.Position;
   TimeDifferenceA            = source.TimeDifferenceA;
   TimeDifferenceB            = source.TimeDifferenceB;
   SpeedOverGroundKnots       = source.SpeedOverGroundKnots;
   TrackMadeGoodDegreesTrue   = source.TrackMadeGoodDegreesTrue;
   MagneticVariation          = source.MagneticVariation;
   MagneticVariationDirection = source.MagneticVariationDirection;

   return( *this );
}
