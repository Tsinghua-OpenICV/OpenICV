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
** $Workfile: lcd.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:44p $
*/



LCD::LCD()
{
   Mnemonic = "LCD";
   Empty();
}

LCD::~LCD()
{
   //Mnemonic.Empty();
   Empty();
}

void LCD::Empty( void )
{
   GroupRepetitionInterval = 0;
   Master.Empty();
   Secondary1.Empty();
   Secondary2.Empty();
   Secondary3.Empty();
   Secondary4.Empty();
   Secondary5.Empty();
}

BOOL LCD::Parse( const SENTENCE& sentence )
{
   /*
   ** LCD - Loran-C Signal Data
   **
   **        1    2   3   4   5   6   7   8   9   10  11  12  13  14
   **        |    |   |   |   |   |   |   |   |   |   |   |   |   |
   ** $--LCD,xxxx,xxx,xxx,xxx,xxx,xxx,xxx,xxx,xxx,xxx,xxx,xxx,xxx*hh<CR><LF>
   **
   **  1) Group Repetition Interval (GRI) Microseconds/10
   **  2) Master Relative SNR
   **  3) Master Relative ECD
   **  4) Time Difference 1 Microseconds
   **  5) Time Difference 1 Signal Status
   **  6) Time Difference 2 Microseconds
   **  7) Time Difference 2 Signal Status
   **  8) Time Difference 3 Microseconds
   **  9) Time Difference 3 Signal Status
   ** 10) Time Difference 4 Microseconds
   ** 11) Time Difference 4 Signal Status
   ** 12) Time Difference 5 Microseconds
   ** 13) Time Difference 5 Signal Status
   ** 14) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 14 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   GroupRepetitionInterval = sentence.Integer( 1 );
   Master.Parse( 2, sentence );
   Secondary1.Parse( 4, sentence );
   Secondary2.Parse( 6, sentence );
   Secondary3.Parse( 8, sentence );
   Secondary4.Parse( 10, sentence );
   Secondary5.Parse( 12, sentence );

   return( TRUE );
}

BOOL LCD::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += GroupRepetitionInterval;
   Master.Write( sentence );
   Secondary1.Write( sentence );
   Secondary2.Write( sentence );
   Secondary3.Write( sentence );
   Secondary4.Write( sentence );
   Secondary5.Write( sentence );

   sentence.Finish();

   return( TRUE );
}

const LCD& LCD::operator = ( const LCD& source )
{
   GroupRepetitionInterval = source.GroupRepetitionInterval;
   Master                  = source.Master;
   Secondary1              = source.Secondary1;
   Secondary2              = source.Secondary2;
   Secondary3              = source.Secondary3;
   Secondary4              = source.Secondary4;
   Secondary5              = source.Secondary5;

   return( *this );
}
