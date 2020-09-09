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
** $Workfile: glc.cpp $
** $Revision: 5 $
** $Modtime: 10/10/98 2:47p $
*/



GLC::GLC()
{
   Mnemonic = "GLC";
   Empty();
}

GLC::~GLC()
{
   //Mnemonic.Empty();
   Empty();
}

void GLC::Empty( void )
{
   GroupRepetitionInterval = 0;
   MasterTOA.Empty();
   TimeDifference1.Empty();
   TimeDifference2.Empty();
   TimeDifference3.Empty();
   TimeDifference4.Empty();
   TimeDifference5.Empty();
}

BOOL GLC::Parse( const SENTENCE& sentence )
{
   /*
   ** GLC - Geographic Position, Loran-C
   **                                           12    14
   **        1    2   3 4   5 6   7 8   9 10  11|   13|
   **        |    |   | |   | |   | |   | |   | |   | |
   ** $--GLC,xxxx,x.x,a,x.x,a,x.x,a.x,x,a,x.x,a,x.x,a*hh<CR><LF>
   **
   **  1) Group Repetition Interval (GRI) Microseconds/10
   **  2) Master TOA Microseconds
   **  3) Master TOA Signal Status
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
   MasterTOA.Parse( 2, sentence );
   TimeDifference1.Parse( 4, sentence );
   TimeDifference2.Parse( 6, sentence );
   TimeDifference3.Parse( 8, sentence );
   TimeDifference4.Parse( 10, sentence );
   TimeDifference5.Parse( 12, sentence );

   return( TRUE );
}

BOOL GLC::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += GroupRepetitionInterval;
   MasterTOA.Write( sentence );
   TimeDifference1.Write( sentence );
   TimeDifference2.Write( sentence );
   TimeDifference3.Write( sentence );
   TimeDifference4.Write( sentence );
   TimeDifference5.Write( sentence );

   sentence.Finish();

   return( TRUE );
}

const GLC& GLC::operator = ( const GLC& source )
{
   GroupRepetitionInterval = source.GroupRepetitionInterval;
   MasterTOA               = source.MasterTOA;
   TimeDifference1         = source.TimeDifference1;
   TimeDifference2         = source.TimeDifference2;
   TimeDifference3         = source.TimeDifference3;
   TimeDifference4         = source.TimeDifference4;
   TimeDifference5         = source.TimeDifference5;

   return( *this );
}
