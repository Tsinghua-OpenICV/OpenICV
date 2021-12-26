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
** $Workfile: bod.cpp $
** $Revision: 6 $
** $Modtime: 10/10/98 2:55p $
*/

#ifdef _MSC_VER
#   pragma warning(disable:4996)
#endif // _MSC_VER

BOD::BOD()
{
   Mnemonic = "BOD";
   Empty();
}

BOD::~BOD()
{
   //Mnemonic.Empty();
   Empty();
}

void BOD::Empty( void )
{
   BearingTrue     = 0.0;
   BearingMagnetic = 0.0;
   //To.Empty();
   //From.Empty();
}

BOOL BOD::Parse( const SENTENCE& sentence )
{
   /*
   ** BOD - Bearing - Origin Waypoint to Destination Waypoint
   **
   **        1   2 3   4 5    6    7
   **        |   | |   | |    |    |
   ** $--BOD,x.x,T,x.x,M,c--c,c--c*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Bearing Degrees, TRUE
   **  2) T = True
   **  3) Bearing Degrees, Magnetic
   **  4) M = Magnetic
   **  5) TO Waypoint
   **  6) FROM Waypoint
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

   BearingTrue     = sentence.Double( 1 );
   BearingMagnetic = sentence.Double( 3 );
   To              = sentence.Field( 5 );
   From            = sentence.Field( 6 );

   return( TRUE );
}

QString BOD::PlainEnglish( void ) const
{
   QString return_string;

   //return_string.Empty();

   return_string = "You are bearing ";

   char temp_string [ 22 ];
   
   ::sprintf( temp_string, "%5.2f", BearingTrue );

   return_string += temp_string;
   return_string += " degrees true (";

   ::sprintf( temp_string, "%5.2f", BearingMagnetic );

   return_string += temp_string;
   return_string += " degrees magnetic).";

   return( return_string );
}

BOOL BOD::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += BearingTrue;
   sentence += "T";
   sentence += BearingMagnetic;
   sentence += "M";
   sentence += To;
   sentence += From;

   sentence.Finish();

   return( TRUE );
}

const BOD& BOD::operator = ( const BOD& source )
{
   BearingTrue     = source.BearingTrue;
   BearingMagnetic = source.BearingMagnetic;
   To              = source.To;
   From            = source.From;

   return( *this );
}
