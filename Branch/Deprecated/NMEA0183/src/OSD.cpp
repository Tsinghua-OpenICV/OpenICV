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
** $Workfile: osd.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:42p $
*/



OSD::OSD()
{
   Mnemonic = "OSD";
   Empty();
}

OSD::~OSD()
{
   //Mnemonic.Empty();
   Empty();
}

void OSD::Empty( void )
{
   HeadingDegreesTrue       = 0.0;
   IsHeadingValid           = NMEA_Unknown;
   VesselCourseDegreesTrue  = 0.0;
   VesselCourseReference    = ReferenceUnknown;
   VesselSpeed              = 0.0;
   VesselSpeedReference     = ReferenceUnknown;
   VesselSetDegreesTrue     = 0.0;
   VesselDriftSpeed         = 0.0;
   //VesselDriftSpeedUnits.Empty();
}

BOOL OSD::Parse( const SENTENCE& sentence )
{
   /*
   ** OSD - Own Ship Data
   **
   **        1   2 3   4 5   6 7   8   9 10
   **        |   | |   | |   | |   |   | |
   ** $--OSD,x.x,A,x.x,a,x.x,a,x.x,x.x,a*hh<CR><LF>
   **
   **  1) Heading, degrees true
   **  2) Status, A = Data Valid
   **  3) Vessel Course, degrees True
   **  4) Course Reference
   **  5) Vessel Speed
   **  6) Speed Reference
   **  7) Vessel Set, degrees True
   **  8) Vessel drift (speed)
   **  9) Speed Units
   ** 10) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 10 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   HeadingDegreesTrue       = sentence.Double( 1 );
   IsHeadingValid           = sentence.Boolean( 2 );
   VesselCourseDegreesTrue  = sentence.Double( 3 );
   VesselCourseReference    = sentence.Reference( 4 );
   VesselSpeed              = sentence.Double( 5 );
   VesselSpeedReference     = sentence.Reference( 6 );
   VesselSetDegreesTrue     = sentence.Double( 7 );
   VesselDriftSpeed         = sentence.Double( 8 );
   VesselDriftSpeedUnits    = sentence.Field( 9 );

   return( TRUE );
}

BOOL OSD::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += HeadingDegreesTrue;
   sentence += IsHeadingValid;
   sentence += VesselCourseDegreesTrue;
   sentence += VesselCourseReference;
   sentence += VesselSpeed;
   sentence += VesselSpeedReference;
   sentence += VesselSetDegreesTrue;
   sentence += VesselDriftSpeed;
   sentence += VesselDriftSpeedUnits;

   sentence.Finish();

   return( TRUE );
}

const OSD& OSD::operator = ( const OSD& source )
{
   HeadingDegreesTrue       = source.HeadingDegreesTrue;
   IsHeadingValid           = source.IsHeadingValid;
   VesselCourseDegreesTrue  = source.VesselCourseDegreesTrue;
   VesselCourseReference    = source.VesselCourseReference;
   VesselSpeed              = source.VesselSpeed;
   VesselSpeedReference     = source.VesselSpeedReference;
   VesselSetDegreesTrue     = source.VesselSetDegreesTrue;
   VesselDriftSpeed         = source.VesselDriftSpeed;
   VesselDriftSpeedUnits    = source.VesselDriftSpeedUnits;

   return( *this );
}
