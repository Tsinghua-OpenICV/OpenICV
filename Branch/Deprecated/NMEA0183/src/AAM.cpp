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
** $Workfile: aam.cpp $
** $Revision: 5 $
** $Modtime: 10/10/98 2:40p $
*/


AAM::AAM()
{
   Mnemonic = "AAM";
   Empty();
}

AAM::~AAM()
{
   //Mnemonic.Empty();
   Empty();
}

void AAM::Empty( void )
{
   IsArrivalCircleEntered = NMEA_Unknown;
   IsPerpendicular        = NMEA_Unknown;
   CircleRadius           = 0.0;
   //WaypointID.Empty();
}

BOOL AAM::Parse( const SENTENCE& sentence )
{
   /*
   ** AAM - Waypoint Arrival Alarm
   **
   **        1 2 3   4 5    6
   **        | | |   | |    |
   ** $--AAM,A,A,x.x,N,c--c*hh<CR><LF>
   **
   ** 1) Status, A = Arrival circle entered
   ** 2) Status, A = perpendicular passed at waypoint
   ** 3) Arrival circle radius
   ** 4) Units of radius, nautical miles
   ** 5) Waypoint ID
   ** 6) Checksum
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

   IsArrivalCircleEntered = sentence.Boolean( 1 );
   IsPerpendicular        = sentence.Boolean( 2 );
   CircleRadius           = sentence.Double( 3 );
   WaypointID             = sentence.Field( 5 );

   return( TRUE );
}

BOOL AAM::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += IsArrivalCircleEntered;
   sentence += IsPerpendicular;
   sentence += CircleRadius;
   sentence += "N";
   sentence += WaypointID;

   sentence.Finish();

   return( TRUE );
}

const AAM& AAM::operator = ( const AAM& source )
{
   IsArrivalCircleEntered = source.IsArrivalCircleEntered;
   IsPerpendicular        = source.IsPerpendicular;
   CircleRadius           = source.CircleRadius;
   WaypointID             = source.WaypointID;

   return( *this );
}
