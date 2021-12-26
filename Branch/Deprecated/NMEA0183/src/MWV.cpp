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
** $Workfile: mwv.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:42p $
*/



MWV::MWV()
{
   Mnemonic = "MWV";
   Empty();
}

MWV::~MWV()
{
   //Mnemonic.Empty();
   Empty();
}

void MWV::Empty( void )
{
   WindAngle   = 0.0;
   //Reference.Empty();
   WindSpeed   = 0.0;
   //WindSpeedUnits.Empty();
   IsDataValid = NMEA_Unknown;
}

BOOL MWV::Parse( const SENTENCE& sentence )
{
   /*
   ** MWV - Wind Speed and Angle
   **
   **        1   2 3   4 5
   **        |   | |   | |
   ** $--MWV,x.x,a,x.x,a*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Wind Angle, 0 to 360 degrees
   **  2) Reference, R = Relative, T = True
   **  3) Wind Speed
   **  4) Wind Speed Units, K/M/N
   **  5) Status, A = Data Valid
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

   WindAngle      = sentence.Double( 1 );
   Reference      = sentence.Field( 2 );
   WindSpeed      = sentence.Double( 3 );
   WindSpeedUnits = sentence.Field( 4 );
   IsDataValid    = sentence.Boolean( 5 );

   return( TRUE );
}

BOOL MWV::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += WindAngle;
   sentence += Reference;
   sentence += WindSpeed;
   sentence += WindSpeedUnits;
   sentence += IsDataValid;

   sentence.Finish();

   return( TRUE );
}

const MWV& MWV::operator = ( const MWV& source )
{
   WindAngle      = source.WindAngle;
   Reference      = source.Reference;
   WindSpeed      = source.WindSpeed;
   WindSpeedUnits = source.WindSpeedUnits;
   IsDataValid    = source.IsDataValid;

   return( *this );
}
