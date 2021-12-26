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
** $Workfile: gxa.cpp $
** $Revision: 5 $
** $Modtime: 10/10/98 2:44p $
*/



GXA::GXA()
{
   Mnemonic = "GXA";
   Empty();
}

GXA::~GXA()
{
   //Mnemonic.Empty();
   Empty();
}

void GXA::Empty( void )
{
   //UTCTime.Empty();
   Position.Empty();
   //WaypointID.Empty();
   SatelliteNumber = 0;
}

BOOL GXA::Parse( const SENTENCE& sentence )
{
   /*
   ** GXA - TRANSIT Position - Latitude/Longitude
   ** Location and time of TRANSIT fix at waypoint
   **
   **        1         2       3 4        5 6    7 8
   **        |         |       | |        | |    | |
   ** $--GXA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,c--c,X*hh<CR><LF>
   **
   ** 1) UTC of position fix
   ** 2) Latitude
   ** 3) East or West
   ** 4) Longitude
   ** 5) North or South
   ** 6) Waypoint ID
   ** 7) Satelite number
   ** 8) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 8 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   UTCTime         = sentence.Field( 1 );
   Time            = sentence.Time( 1 );
   Position.Parse( 2, 3, 4, 5, sentence );
   WaypointID      = sentence.Field( 6 );
   SatelliteNumber = (WORD) sentence.Integer( 7 );

   return( TRUE );
}

BOOL GXA::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += UTCTime;
   sentence += Position;
   sentence += WaypointID;
   sentence += Hex( SatelliteNumber ); // Thanks to Chuck Shannon, cshannon@imtn.tpd.dsccc.com

   sentence.Finish();

   return( TRUE );
}

const GXA& GXA::operator = ( const GXA& source )
{
   UTCTime         = source.UTCTime;
   Time            = source.Time;
   Position        = source.Position;
   WaypointID      = source.WaypointID;
   SatelliteNumber = source.SatelliteNumber;

   return( *this );
}
