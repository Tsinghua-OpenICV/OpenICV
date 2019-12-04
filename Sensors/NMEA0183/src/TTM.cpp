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
** $Workfile: ttm.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:38p $
*/



TTM::TTM()
{
   Mnemonic = "TTM";
   Empty();
}

TTM::~TTM()
{
   //Mnemonic.Empty();
   Empty();
}

void TTM::Empty( void )
{
   TargetNumber                            = 0;
   TargetDistance                          = 0.0;
   BearingFromOwnShip                      = 0.0;
   //BearingUnits.Empty();
   TargetSpeed                             = 0.0;
   TargetCourse                            = 0.0;
   //TargetCourseUnits.Empty();
   DistanceOfClosestPointOfApproach        = 0.0;
   NumberOfMinutesToClosestPointOfApproach = 0.0;
   //Increasing.Empty();
   //TargetName.Empty();
   TargetStatus                            = TargetUnknown;
}

BOOL TTM::Parse( const SENTENCE& sentence )
{
   /*
   ** TTM - Tracked Target Message
   **
   **                                         11     13
   **        1  2   3   4 5   6   7 8   9   10|    12| 14
   **        |  |   |   | |   |   | |   |   | |    | | |
   ** $--TTM,xx,x.x,x.x,a,x.x,x.x,a,x.x,x.x,a,c--c,a,a*hh<CR><LF>
   **
   **  1) Target Number
   **  2) Target Distance
   **  3) Bearing from own ship
   **  4) Bearing Units
   **  5) Target speed
   **  6) Target Course
   **  7) Course Units
   **  8) Distance of closest-point-of-approach
   **  9) Time until closest-point-of-approach "-" means increasing
   ** 10) "-" means increasing
   ** 11) Target name
   ** 12) Target Status
   ** 13) Reference Target
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

   TargetNumber                            = sentence.Integer( 1 );
   TargetDistance                          = sentence.Double( 2 );
   BearingFromOwnShip                      = sentence.Double( 3 );
   BearingUnits                            = sentence.Field( 4 );
   TargetSpeed                             = sentence.Double( 5 );
   TargetCourse                            = sentence.Double( 6 );
   TargetCourseUnits                       = sentence.Field( 7 );
   DistanceOfClosestPointOfApproach        = sentence.Double( 8 );
   NumberOfMinutesToClosestPointOfApproach = sentence.Double( 9 );
   Increasing                              = sentence.Field( 10 );
   TargetName                              = sentence.Field( 11 );
   
   QString field_data = sentence.Field( 12 );

   if ( field_data == "L" )
   {
      TargetStatus = TargetLost;
   }
   else if ( field_data == "Q" )
   {
      TargetStatus = TargetQuery;
   }
   else if ( field_data == "T" )
   {
      TargetStatus = TargetTracking;
   }
   else
   {
      TargetStatus = TargetUnknown;
   }

   ReferenceTarget = sentence.Field( 13 );

   return( TRUE );
}

BOOL TTM::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += TargetNumber;
   sentence += TargetDistance;
   sentence += BearingFromOwnShip;
   sentence += BearingUnits;
   sentence += TargetSpeed;
   sentence += TargetCourse;
   sentence += TargetCourseUnits;
   sentence += DistanceOfClosestPointOfApproach;
   sentence += NumberOfMinutesToClosestPointOfApproach;
   sentence += Increasing;
   sentence += TargetName;
   
   switch( TargetStatus )
   {
      case TargetLost:

         sentence += "L";
         break;

      case TargetQuery:

         sentence += "Q";
         break;

      case TargetTracking:

         sentence += "T";
         break;

      default:

         sentence += "";
         break;
   }

   sentence += ReferenceTarget;

   sentence.Finish();

   return( TRUE );
}

const TTM& TTM::operator = ( const TTM& source )
{
   TargetNumber                            = source.TargetNumber;
   TargetDistance                          = source.TargetDistance;
   BearingFromOwnShip                      = source.BearingFromOwnShip;
   BearingUnits                            = source.BearingUnits;
   TargetSpeed                             = source.TargetSpeed;
   TargetCourse                            = source.TargetCourse;
   TargetCourseUnits                       = source.TargetCourseUnits;
   DistanceOfClosestPointOfApproach        = source.DistanceOfClosestPointOfApproach;
   NumberOfMinutesToClosestPointOfApproach = source.NumberOfMinutesToClosestPointOfApproach;
   Increasing                              = source.Increasing;
   TargetName                              = source.TargetName;
   TargetStatus                            = source.TargetStatus;

   return( *this );
}
