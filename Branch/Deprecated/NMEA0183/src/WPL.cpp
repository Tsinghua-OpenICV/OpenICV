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
** $Workfile: wpl.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:39p $
*/



WPL::WPL()
{
   Mnemonic = "WPL";
   Empty();
}

WPL::~WPL()
{
   //Mnemonic.Empty();
   Empty();
}

void WPL::Empty( void )
{
   Position.Empty();
   //To.Empty();
}

BOOL WPL::Parse( const SENTENCE& sentence )
{
   /*
   ** WPL - Waypoint Location
   **
   **        +-------------------------------- 1) Latitude
   **        |       +------------------------ 2) N or S (North or South)
   **        |       | +---------------------- 3) Longitude
   **        |       | |        +------------- 4) E or W (East or West)
   **        |       | |        | +----------- 5) Waypoint name
   **        |       | |        | |    +-------6) Checksum     
   **        |       | |        | |    |
   ** $--WPL,llll.ll,a,yyyyy.yy,a,c--c*hh<CR><LF>
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 6 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   Position.Parse( 1, 2, 3, 4, sentence );
   To = sentence.Field( 5 );

   return( TRUE );
}

BOOL WPL::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += Position;
   sentence += To;

   sentence.Finish();

   return( TRUE );
}

const WPL& WPL::operator = ( const WPL& source )
{
   Position = source.Position;
   To       = source.To;

   return( *this );
}
