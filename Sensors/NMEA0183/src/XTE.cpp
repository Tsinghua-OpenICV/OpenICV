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
** $Workfile: xte.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:39p $
*/


XTE::XTE()
{
   Mnemonic = "XTE";
   Empty();
}

XTE::~XTE()
{
   //Mnemonic.Empty();
   Empty();
}

void XTE::Empty( void )
{
   CrossTrackErrorMagnitude = 0.0;
   DirectionToSteer         = LR_Unknown;
   //CrossTrackUnits.Empty();
}

BOOL XTE::Parse( const SENTENCE& sentence )
{
   QString field_data;

   /*
   ** XTE - Cross-Track Error, Measured
   **
   **        1 2 3   4 5  6
   **        | | |   | |  |
   ** $--XTE,A,A,x.x,a,N,*hh<CR><LF>
   **
   **  1) Status
   **     V = LORAN-C Blink or SNR warning
   **     V = general warning flag or other navigation systems when a reliable
   **         fix is not available
   **  2) Status
   **     V = Loran-C Cycle Lock warning flag
   **     A = OK or not used
   **  3) Cross Track Error Magnitude
   **  4) Direction to steer, L or R
   **  5) Cross Track Units, N = Nautical Miles
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

   /*
   ** Line has already been checked for checksum validity
   */

   IsLoranBlinkOK           = sentence.Boolean( 1 );
   IsLoranCCycleLockOK      = sentence.Boolean( 2 );
   CrossTrackErrorMagnitude = sentence.Double( 3 );
   DirectionToSteer         = sentence.LeftOrRight( 4 );
   CrossTrackUnits          = sentence.Field( 5 );

   return( TRUE );
}

BOOL XTE::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += IsLoranBlinkOK;
   sentence += IsLoranCCycleLockOK;
   sentence += CrossTrackErrorMagnitude;
   sentence += DirectionToSteer;
   sentence += CrossTrackUnits;

   sentence.Finish();

   return( TRUE );
}

const XTE& XTE::operator = ( const XTE& source )
{
   IsLoranBlinkOK           = source.IsLoranBlinkOK;
   IsLoranCCycleLockOK      = source.IsLoranCCycleLockOK;
   CrossTrackErrorMagnitude = source.CrossTrackErrorMagnitude;
   DirectionToSteer         = source.DirectionToSteer;
   CrossTrackUnits          = source.CrossTrackUnits;

   return( *this );
}
