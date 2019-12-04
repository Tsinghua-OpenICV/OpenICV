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
** $Workfile: vbw.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:38p $
*/


VBW::VBW()
{
   Mnemonic = "VBW";
   Empty();
}

VBW::~VBW()
{
   //Mnemonic.Empty();
   Empty();
}

void VBW::Empty( void )
{
   LongitudinalWaterSpeed  = 0.0;
   TransverseWaterSpeed    = 0.0;
   IsWaterSpeedValid       = NMEA_Unknown;
   LongitudinalGroundSpeed = 0.0;
   TransverseGroundSpeed   = 0.0;
   IsGroundSpeedValid      = NMEA_Unknown;
}

BOOL VBW::Parse( const SENTENCE& sentence )
{
   /*
   ** VBW - Dual Ground/Water Speed
   **
   **        1   2   3 4   5   6 7
   **        |   |   | |   |   | |
   ** $--VBW,x.x,x.x,A,x.x,x.x,A*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Longitudinal water speed, "-" means astern
   **  2) Transverse water speed, "-" means port
   **  3) Status, A = Data Valid
   **  4) Longitudinal ground speed, "-" means astern
   **  5) Transverse ground speed, "-" means port
   **  6) Status, A = Data Valid
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

   LongitudinalWaterSpeed  = sentence.Double( 1 );
   TransverseWaterSpeed    = sentence.Double( 2 );
   IsWaterSpeedValid       = sentence.Boolean( 3 );
   LongitudinalGroundSpeed = sentence.Double( 4 );
   TransverseGroundSpeed   = sentence.Double( 5 );
   IsGroundSpeedValid      = sentence.Boolean( 6 );

   return( TRUE );
}

BOOL VBW::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += LongitudinalWaterSpeed;
   sentence += TransverseWaterSpeed;
   sentence += IsWaterSpeedValid;
   sentence += LongitudinalGroundSpeed;
   sentence += TransverseGroundSpeed;
   sentence += IsGroundSpeedValid;

   sentence.Finish();

   return( TRUE );
}

const VBW& VBW::operator = ( const VBW& source )
{
   LongitudinalWaterSpeed  = source.LongitudinalWaterSpeed;
   TransverseWaterSpeed    = source.TransverseWaterSpeed;
   IsWaterSpeedValid       = source.IsWaterSpeedValid;
   LongitudinalGroundSpeed = source.LongitudinalGroundSpeed;
   TransverseGroundSpeed   = source.TransverseGroundSpeed;
   IsGroundSpeedValid      = source.IsGroundSpeedValid;

   return( *this );
}
