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
** $Workfile: dcn.cpp $
** $Revision: 5 $
** $Modtime: 10/10/98 2:46p $
*/


DCN::DCN()
{
   Mnemonic = "DCN";
   Empty();
}

DCN::~DCN()
{
   //Mnemonic.Empty();
   Empty();
}

void DCN::Empty( void )
{
   DeccaChainID                     = 0;
   Red.Empty();
   Green.Empty();
   Purple.Empty();
   RedLineNavigationUse             = NMEA_Unknown;
   GreenLineNavigationUse           = NMEA_Unknown;
   PurpleLineNavigationUse          = NMEA_Unknown;
   PositionUncertaintyNauticalMiles = 0.0;
   Basis                            = BasisUnknown;
}

BOOL DCN::Parse( const SENTENCE& sentence )
{
   /*
   ** DCN - Decca Position
   **                                      11  13      16
   **        1  2  3   4 5  6   7 8  9   10| 12| 14  15| 17
   **        |  |  |   | |  |   | |  |   | | | | |   | | |
   ** $--DCN,xx,cc,x.x,A,cc,x.x,A,cc,x.x,A,A,A,A,x.x,N,x*hh<CR><LF>
   **
   **  1) Decca chain identifier
   **  2) Red Zone Identifier
   **  3) Red Line Of Position
   **  4) Red Master Line Status
   **  5) Green Zone Identifier
   **  6) Green Line Of Position
   **  7) Green Master Line Status
   **  8) Purple Zone Identifier
   **  9) Purple Line Of Position
   ** 10) Purple Master Line Status
   ** 11) Red Line Navigation Use
   ** 12) Green Line Navigation Use
   ** 13) Purple Line Navigation Use
   ** 14) Position Uncertainity
   ** 15) N = Nautical Miles
   ** 16) Fix Data Basis
   **     1 = Normal Pattern
   **     2 = Lane Identification Pattern
   **     3 = Lane Identification Transmissions
   ** 17) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 17 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   DeccaChainID                     = sentence.Integer( 1 );
   Red.Parse( 2, sentence );
   Green.Parse( 5, sentence );
   Purple.Parse( 8, sentence );
   RedLineNavigationUse             = sentence.Boolean( 11 );
   GreenLineNavigationUse           = sentence.Boolean( 12 );
   PurpleLineNavigationUse          = sentence.Boolean( 13 );
   PositionUncertaintyNauticalMiles = sentence.Double( 14 );

   int temp_integer = sentence.Integer( 16 );

   switch( temp_integer )
   {
      case 1:

         Basis = NormalPatternBasis;
         break;

      case 2:

         Basis = LaneIdentificationPatternBasis;
         break;

      case 3:

         Basis = LaneIdentificationTransmissionsBasis;
         break;

      default:

         Basis = BasisUnknown;
         break;
   }

   return( TRUE );
}

BOOL DCN::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += DeccaChainID;
   Red.Write( sentence );
   Green.Write( sentence );
   Purple.Write( sentence );
   sentence += RedLineNavigationUse;
   sentence += GreenLineNavigationUse;
   sentence += PurpleLineNavigationUse;
   sentence += PositionUncertaintyNauticalMiles;
   sentence += "N";

   switch( Basis )
   {
      case NormalPatternBasis:

         sentence += 1;
         break;

      case LaneIdentificationPatternBasis:

         sentence += 2;
         break;

      case LaneIdentificationTransmissionsBasis:

         sentence += 3;
         break;

      default:

         sentence += "";
         break;
   }

   sentence.Finish();

   return( TRUE );
}

const DCN& DCN::operator = ( const DCN& source )
{
   DeccaChainID                     = source.DeccaChainID;
   Red                              = source.Red;
   Green                            = source.Green;
   Purple                           = source.Purple;
   RedLineNavigationUse             = source.RedLineNavigationUse;
   GreenLineNavigationUse           = source.GreenLineNavigationUse;
   PurpleLineNavigationUse          = source.PurpleLineNavigationUse;
   PositionUncertaintyNauticalMiles = source.PositionUncertaintyNauticalMiles;
   Basis                            = source.Basis;
   
   return( *this );
}
