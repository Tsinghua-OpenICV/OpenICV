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
** $Workfile: rmb.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:43p $
*/


RMB::RMB()
{
   Mnemonic = "RMB";
   Empty();
}

RMB::~RMB()
{
   //Mnemonic.Empty();
   Empty();
}

void RMB::Empty( void )
{
   IsDataValid                     = NMEA_Unknown;
   CrossTrackError                 = 0.0;
   DirectionToSteer                = LR_Unknown;
   //To.Empty();
   //From.Empty();
   DestinationPosition;
   RangeToDestinationNauticalMiles = 0.0;
   BearingToDestinationDegreesTrue = 0.0;
   DestinationClosingVelocityKnots = 0.0;
   IsArrivalCircleEntered          = NMEA_Unknown;
}

BOOL RMB::Parse( const SENTENCE& sentence )
{
   /*
   ** RMB - Recommended Minimum Navigation Information
   **                                                             14
   **        1 2   3 4    5    6       7 8        9 10  11  12  13|
   **        | |   | |    |    |       | |        | |   |   |   | |
   ** $--RMB,A,x.x,a,c--c,c--c,llll.ll,a,yyyyy.yy,a,x.x,x.x,x.x,A*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Status, V = Navigation receiver warning
   **  2) Cross Track error - nautical miles
   **  3) Direction to Steer, Left or Right
   **  4) TO Waypoint ID
   **  5) FROM Waypoint ID
   **  6) Destination Waypoint Latitude
   **  7) N or S
   **  8) Destination Waypoint Longitude
   **  9) E or W
   ** 10) Range to destination in nautical miles
   ** 11) Bearing to destination in degrees True
   ** 12) Destination closing velocity in knots
   ** 13) Arrival Status, A = Arrival Circle Entered
   ** 14) Checksum
   */

   /*
   ** First we check the checksum...
   */

   NMEA0183_BOOLEAN check = sentence.IsChecksumBad( 14 );

   if ( check == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   }
   
   if ( check == NMEA_Unknown )
   {
      SetErrorMessage( "Missing Checksum" );
      return( FALSE );
   } 

   IsDataValid                     = sentence.Boolean( 1 );
   CrossTrackError                 = sentence.Double( 2 );
   DirectionToSteer                = sentence.LeftOrRight( 3 );
   From                            = sentence.Field( 4 );
   To                              = sentence.Field( 5 );
   DestinationPosition.Parse( 6, 7, 8, 9, sentence );
   RangeToDestinationNauticalMiles = sentence.Double( 10 );
   BearingToDestinationDegreesTrue = sentence.Double( 11 );
   DestinationClosingVelocityKnots = sentence.Double( 12 );
   IsArrivalCircleEntered          = sentence.Boolean( 13 );

   return( TRUE );
}

BOOL RMB::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += IsDataValid;
   sentence += CrossTrackError;
   sentence += DirectionToSteer;
   sentence += From;
   sentence += To;
   sentence += DestinationPosition;
   sentence += RangeToDestinationNauticalMiles;
   sentence += BearingToDestinationDegreesTrue;
   sentence += DestinationClosingVelocityKnots;
   sentence += IsArrivalCircleEntered;

   sentence.Finish();

   return( TRUE );
}

const RMB& RMB::operator = ( const RMB& source )
{
   IsDataValid                     = source.IsDataValid;
   CrossTrackError                 = source.CrossTrackError;
   DirectionToSteer                = source.DirectionToSteer;
   From                            = source.From;
   To                              = source.To;
   DestinationPosition             = source.DestinationPosition;
   RangeToDestinationNauticalMiles = source.RangeToDestinationNauticalMiles;
   BearingToDestinationDegreesTrue = source.BearingToDestinationDegreesTrue;
   DestinationClosingVelocityKnots = source.DestinationClosingVelocityKnots;
   IsArrivalCircleEntered          = source.IsArrivalCircleEntered;

  return( *this );
}
