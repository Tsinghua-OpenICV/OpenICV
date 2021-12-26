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
** $Workfile: gtd.cpp $
** $Revision: 5 $
** $Modtime: 10/10/98 2:48p $
*/



GTD::GTD()
{
   Mnemonic = "GTD";
   Empty();
}

GTD::~GTD()
{
   //Mnemonic.Empty();
   Empty();
}

void GTD::Empty( void )
{
   //TimeDifference1.Empty();
   //TimeDifference2.Empty();
   //TimeDifference3.Empty();
   //TimeDifference4.Empty();
   //TimeDifference5.Empty();
}

BOOL GTD::Parse( const SENTENCE& sentence )
{
   /*
   ** GTD - Geographical Position, Loran-C TDs
   **
   **        1   2   3   4   5   6
   **        |   |   |   |   |   |
   ** $--GTD,x.x,x.x,x.x,x,x,x.x*hh<CR><LF>
   **
   **  1) Time Difference 1 Microseconds
   **  2) Time Difference 2 Microseconds
   **  3) Time Difference 3 Microseconds
   **  4) Time Difference 4 Microseconds
   **  5) Time Difference 5 Microseconds
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

   TimeDifference1 = sentence.Field( 1 );
   TimeDifference2 = sentence.Field( 2 );
   TimeDifference3 = sentence.Field( 3 );
   TimeDifference4 = sentence.Field( 4 );
   TimeDifference5 = sentence.Field( 5 );

   return( TRUE );
}

BOOL GTD::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += TimeDifference1;
   sentence += TimeDifference2;
   sentence += TimeDifference3;
   sentence += TimeDifference4;
   sentence += TimeDifference5;

   sentence.Finish();

   return( TRUE );
}

const GTD& GTD::operator = ( const GTD& source )
{
   TimeDifference1 = source.TimeDifference1;
   TimeDifference2 = source.TimeDifference2;
   TimeDifference3 = source.TimeDifference3;
   TimeDifference4 = source.TimeDifference4;
   TimeDifference5 = source.TimeDifference5;

   return( *this );
}
