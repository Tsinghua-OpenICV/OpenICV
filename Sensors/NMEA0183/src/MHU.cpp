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
** $Workfile: mhu.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:42p $
*/

/*
** This Sentence Not Recommended For New Designs
** XDR is recommended.
*/



MHU::MHU()
{
   Mnemonic = "MHU";
   Empty();
}

MHU::~MHU()
{
   //Mnemonic.Empty();
   Empty();
}

void MHU::Empty( void )
{
   RelativeHumidityPercent = 0.0;
   AbsoluteHumidityPercent = 0.0;
   DewPointDegreesCelcius  = 0.0;
}

BOOL MHU::Parse( const SENTENCE& sentence )
{
   /*
   ** MHU - Humidity
   **
   **        1   2   3   4 5
   **        |   |   |   | |
   ** $--MHU,x.x,x.x,x.x,C*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Relative Humidity Percent
   **  2) Absolute humidity percent
   **  3) Dew Point
   **  4) C = Degrees Celsius
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

   RelativeHumidityPercent = sentence.Double( 1 );
   AbsoluteHumidityPercent = sentence.Double( 2 );
   DewPointDegreesCelcius  = sentence.Double( 3 );

   return( TRUE );
}

BOOL MHU::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += RelativeHumidityPercent;
   sentence += AbsoluteHumidityPercent;
   sentence += DewPointDegreesCelcius;
   sentence += "C";

   sentence.Finish();

   return( TRUE );
}

const MHU& MHU::operator = ( const MHU& source )
{
   RelativeHumidityPercent  = source.RelativeHumidityPercent;
   AbsoluteHumidityPercent  = source.AbsoluteHumidityPercent;
   DewPointDegreesCelcius   = source.DewPointDegreesCelcius;

   return( *this );
}
