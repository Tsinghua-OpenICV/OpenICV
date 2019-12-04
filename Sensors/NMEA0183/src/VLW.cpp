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
** $Workfile: vlw.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:40p $
*/



VLW::VLW()
{
   Mnemonic = "VLW";
   Empty();
}

VLW::~VLW()
{
   //Mnemonic.Empty();
   Empty();
}

void VLW::Empty( void )
{
   TotalDistanceNauticalMiles      = 0.0;
   DistanceSinceResetNauticalMiles = 0.0;
}

BOOL VLW::Parse( const SENTENCE& sentence )
{
   /*
   ** VLW - Distance Traveled through Water
   **
   **        1   2 3   4 5
   **        |   | |   | |
   ** $--VLW,x.x,N,x.x,N*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Total cumulative distance
   **  2) N = Nautical Miles
   **  3) Distance since Reset
   **  4) N = Nautical Miles
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

   TotalDistanceNauticalMiles      = sentence.Double( 1 );
   DistanceSinceResetNauticalMiles = sentence.Double( 3 );

   return( TRUE );
}

BOOL VLW::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += TotalDistanceNauticalMiles;
   sentence += "N";
   sentence += DistanceSinceResetNauticalMiles;
   sentence += "N";

   sentence.Finish();

   return( TRUE );
}

const VLW& VLW::operator = ( const VLW& source )
{
   TotalDistanceNauticalMiles      = source.TotalDistanceNauticalMiles;
   DistanceSinceResetNauticalMiles = source.DistanceSinceResetNauticalMiles;

   return( *this );
}
