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
** $Workfile: gsv.cpp $
** $Revision: 5 $
** $Modtime: 10/10/98 2:48p $
*/


GSV::GSV()
{
   Mnemonic = "GSV";
   Empty();
}

GSV::~GSV()
{
   //Mnemonic.Empty();
   Empty();
}

void GSV::Empty( void )
{
  
   NumberOfSatellites = 0;

   int index = 0;

   while( index < 36 )
   {
      SatellitesInView[ index ].Empty();
      index++;
   }
}

BOOL GSV::Parse( const SENTENCE& sentence )
{
   /*
   ** GSV - TRANSIT Position - Latitude/Longitude
   ** Location and time of TRANSIT fix at waypoint
   **
   **        1 2 3  4  5  6   7  8  9  10  11 12 13 14  15 16 17 18  19  20
   **        | | |  |  |  |   |  |  |  |   |  |  |  |   |  |  |  |   |   |
   ** $--GSV,x,x,xx,xx,xx,xxx,xx,xx,xx,xxx,xx,xx,xx,xxx,xx,xx,xx,xxx,xx,*hh<CR><LF>
   **
   **  1) Total number of messages, 1-9
   **  2) Message Number, 1-9
   **  3) Total number of satellites in view
   **  4) Satellite Number #1
   **  5) Elevation #1
   **  6) Azimuth, Degrees True #1
   **  7) SNR #1, NULL when not tracking
   **  8) Satellite Number #2
   **  9) Elevation #2
   ** 10) Azimuth, Degrees True #2
   ** 11) SNR #2, NULL when not tracking
   ** 12) Satellite Number #3
   ** 13) Elevation #3
   ** 14) Azimuth, Degrees True #3
   ** 15) SNR #3, NULL when not tracking
   ** 16) Satellite Number #4
   ** 17) Elevation #4
   ** 18) Azimuth, Degrees True #4
   ** 19) SNR #4, NULL when not tracking
   ** 20) Checksum
   */  

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 20 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   }

   Totalmessages = sentence.Integer( 1 );
   //int message_number = sentence.Integer( 2 );
   message_number = sentence.Integer( 2 );

   NumberOfSatellites = sentence.Integer( 3 );

   int index = 0;

   int tmp = NumberOfSatellites - ( message_number - 1 ) * 4;
   int min = ( (4 < tmp) ? 4 : tmp);
   //   while( index < (4 <? (NumberOfSatellites - ( message_number - 1 ) * 4) ) )
   while( index < min )
   {
      SatellitesInView[ ( ( message_number - 1 ) * 4 ) + index ].Parse( ( index * 4 ) + 4, sentence );
      index++;
   }

   return( TRUE );
}

BOOL GSV::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   /*
   ** OK, this is a hack, I'll figure out how to do multiple messages later
   */

   sentence.Finish();

   return( TRUE );
}


const GSV& GSV::operator = ( const GSV& source )
{
   NumberOfSatellites = source.NumberOfSatellites;

   int index = 0;

   while( index < 36 )
   {
      SatellitesInView[ index ] = source.SatellitesInView[ index ];
      index++;
   }
   Totalmessages = source.Totalmessages;
   message_number = source.message_number;
   return( *this );
}


