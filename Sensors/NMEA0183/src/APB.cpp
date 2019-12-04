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
** $Workfile: apb.cpp $
** $Revision: 5 $
** $Modtime: 10/10/98 2:41p $
*/



APB::APB()
{
   Mnemonic = "APB";
   Empty();
}

APB::~APB()
{
   //Mnemonic.Empty();
   Empty();
}

void APB::Empty( void )
{
   CrossTrackErrorMagnitude = 0.0;
   DirectionToSteer         = LR_Unknown;
   //CrossTrackUnits.Empty();
   //To.Empty();
   IsArrivalCircleEntered   = NMEA_Unknown;
   IsPerpendicular          = NMEA_Unknown;
}

BOOL APB::Parse( const SENTENCE& sentence )
{
   QString field_data;

   /*
   ** APB - Autopilot Sentence "B"
   **                                         13    15
   **        1 2 3   4 5 6 7 8   9 10   11  12|   14|
   **        | | |   | | | | |   | |    |   | |   | |
   ** $--APB,A,A,x.x,a,N,A,A,x.x,a,c--c,x.x,a,x.x,a*hh<CR><LF>
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
   **  6) Status
   **     A = Arrival Circle Entered
   **  7) Status
   **     A = Perpendicular passed at waypoint
   **  8) Bearing origin to destination
   **  9) M = Magnetic, T = True
   ** 10) Destination Waypoint ID
   ** 11) Bearing, present position to Destination
   ** 12) M = Magnetic, T = True
   ** 13) Heading to steer to destination waypoint
   ** 14) M = Magnetic, T = True
   ** 15) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 15 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   /*
   ** Line has already been checked for checksum validity
   */

   IsLoranBlinkOK                           = sentence.Boolean( 1 );
   IsLoranCCycleLockOK                      = sentence.Boolean( 2 );
   CrossTrackErrorMagnitude                 = sentence.Double( 3 );
   DirectionToSteer                         = sentence.LeftOrRight( 4 );
   CrossTrackUnits                          = sentence.Field( 5 );
   IsArrivalCircleEntered                   = sentence.Boolean( 6 );
   IsPerpendicular                          = sentence.Boolean( 7 );
   BearingOriginToDestination               = sentence.Double( 8 );
   BearingOriginToDestinationUnits          = sentence.Field( 9 );
   To                                       = sentence.Field( 10 );
   BearingPresentPositionToDestination      = sentence.Double( 11 );
   BearingPresentPositionToDestinationUnits = sentence.Field( 12 );
   HeadingToSteer                           = sentence.Double( 13 );
   HeadingToSteerUnits                      = sentence.Field( 14 );

   return( TRUE );
}

BOOL APB::Write( SENTENCE& sentence )
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
   sentence += IsArrivalCircleEntered;
   sentence += IsPerpendicular;
   sentence += BearingOriginToDestination;
   sentence += BearingOriginToDestinationUnits;
   sentence += To;
   sentence += BearingPresentPositionToDestination;
   sentence += BearingPresentPositionToDestinationUnits;
   sentence += HeadingToSteer;
   sentence += HeadingToSteerUnits;

   sentence.Finish();

   return( TRUE );
}

const APB& APB::operator = ( const APB& source )
{
   IsLoranBlinkOK                           = source.IsLoranBlinkOK;
   IsLoranCCycleLockOK                      = source.IsLoranCCycleLockOK;
   CrossTrackErrorMagnitude                 = source.CrossTrackErrorMagnitude;
   DirectionToSteer                         = source.DirectionToSteer;
   CrossTrackUnits                          = source.CrossTrackUnits;
   IsArrivalCircleEntered                   = source.IsArrivalCircleEntered;
   IsPerpendicular                          = source.IsPerpendicular;
   BearingOriginToDestination               = source.BearingOriginToDestination;
   BearingOriginToDestinationUnits          = source.BearingOriginToDestinationUnits;
   To                                       = source.To;
   BearingPresentPositionToDestination      = source.BearingPresentPositionToDestination;
   BearingPresentPositionToDestinationUnits = source.BearingPresentPositionToDestinationUnits;
   HeadingToSteer                           = source.HeadingToSteer;
   HeadingToSteerUnits                      = source.HeadingToSteerUnits;

   return( *this );
}
