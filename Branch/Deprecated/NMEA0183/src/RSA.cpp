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
** $Workfile: rsa.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:43p $
*/


RSA::RSA()
{
   Mnemonic = "RSA";
   Empty();
}

RSA::~RSA()
{
   //Mnemonic.Empty();
   Empty();
}

void RSA::Empty( void )
{
   Starboard            = 0.0;
   IsStarboardDataValid = NMEA_Unknown;
   Port                 = 0.0;
   IsPortDataValid      = NMEA_Unknown;
}

BOOL RSA::Parse( const SENTENCE& sentence )
{
   /*
   ** RSA - Rudder Sensor Angle
   **
   **        1   2 3   4 5
   **        |   | |   | |
   ** $--RSA,x.x,A,x.x,A*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Starboard (or single) rudder sensor, "-" means Turn To Port
   **  2) Status, A means data is valid
   **  3) Port rudder sensor
   **  4) Status, A means data is valid
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

   Starboard            = sentence.Double(  1 );
   IsStarboardDataValid = sentence.Boolean( 2 );
   Port                 = sentence.Double(  3 );
   IsPortDataValid      = sentence.Boolean( 4 );

   return( TRUE );
}

BOOL RSA::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += Starboard;
   sentence += IsStarboardDataValid;
   sentence += Port;
   sentence += IsPortDataValid;
   
   sentence.Finish();

   return( TRUE );
}

const RSA& RSA::operator = ( const RSA& source )
{
   Starboard            = source.Starboard;
   IsStarboardDataValid = source.IsStarboardDataValid;
   Port                 = source.Port;
   IsPortDataValid      = source.IsPortDataValid;

   return( *this );
}
