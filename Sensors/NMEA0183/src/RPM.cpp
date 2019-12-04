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
** $Workfile: rpm.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:44p $
*/


RPM::RPM()
{
   Mnemonic = "RPM";
   Empty();
}

RPM::~RPM()
{
   //Mnemonic.Empty();
   Empty();
}

void RPM::Empty( void )
{
   //Source.Empty();
   SourceNumber             = 0;
   RevolutionsPerMinute     = 0.0;
   PropellerPitchPercentage = 0.0;
   IsDataValid              = NMEA_Unknown;
}

BOOL RPM::Parse( const SENTENCE& sentence )
{
   /*
   ** RPM - Revolutions
   **
   **        1 2 3   4   5 6
   **        | | |   |   | |
   ** $--RPM,a,x,x.x,x.x,A*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Sourse, S = Shaft, E = Engine
   **  2) Engine or shaft number
   **  3) Speed, Revolutions per minute
   **  4) Propeller pitch, % of maximum, "-" means astern
   **  5) Status, A means data is valid
   **  6) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 6 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   Source                   = sentence.Field( 1 );
   SourceNumber             = sentence.Integer( 2 );
   RevolutionsPerMinute     = sentence.Double( 3 );
   PropellerPitchPercentage = sentence.Double( 4 );
   IsDataValid              = sentence.Boolean( 5 );

   return( TRUE );
}

BOOL RPM::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += Source;
   sentence += SourceNumber;
   sentence += RevolutionsPerMinute;
   sentence += PropellerPitchPercentage;
   sentence += IsDataValid;
   
   sentence.Finish();

   return( TRUE );
}

const RPM& RPM::operator = ( const RPM& source )
{
   Source                   = source.Source;
   SourceNumber             = source.SourceNumber;
   RevolutionsPerMinute     = source.RevolutionsPerMinute;
   PropellerPitchPercentage = source.PropellerPitchPercentage;
   IsDataValid              = source.IsDataValid;

   return( *this );
}
