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
** $Workfile: gsa.cpp $
** $Revision: 5 $
** $Modtime: 10/10/98 2:42p $
*/



GSA::GSA()
{
   Mnemonic = "GSA";
   Empty();
}

GSA::~GSA()
{
   //Mnemonic.Empty();
   Empty();
}

void GSA::Empty( void )
{
   OperatingMode = UnknownOperatingMode;
   FixMode       = FixUnknown;

   int index = 0;

   while( index < 12 )
   {
      SatelliteNumber[ index ] = 0;
      index++;
   }

   PDOP = 0.0;
   HDOP = 0.0;
   VDOP = 0.0;
}

BOOL GSA::Parse( const SENTENCE& sentence )
{
   /*
   ** GSA - GPS DOP and Active Satellites
   **
   **        1 2 3  4  5  6  7  8  9  10 11 12 13 14 15  16  17  18
   **        | | |  |  |  |  |  |  |  |  |  |  |  |  |   |   |   |
   ** $--GSA,a,x,xx,xx,xx,xx,xx,xx,xx,xx,xx,xx,xx,xx,x.x,x.x,x.x*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Operating Mode, A = Automatic, M = Manual
   **  2) Fix Mode, 1 = Fix not available, 2 = 2D, 3 = 3D
   **  3) Satellite PRN #1
   **  4) Satellite PRN #2
   **  5) Satellite PRN #3
   **  6) Satellite PRN #4
   **  7) Satellite PRN #5
   **  8) Satellite PRN #6
   **  9) Satellite PRN #7
   ** 10) Satellite PRN #8
   ** 11) Satellite PRN #9
   ** 12) Satellite PRN #10
   ** 13) Satellite PRN #11
   ** 14) Satellite PRN #12
   ** 15) PDOP
   ** 16) HDOP
   ** 17) VDOP
   ** 18) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 18 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   QString field_data = sentence.Field( 1 );

   if ( field_data == "A" )
   {
      OperatingMode = Automatic;
   }
   else if ( field_data == "M" )
   {
      OperatingMode = Manual;
   }
   else
   {
      OperatingMode = UnknownOperatingMode;
   }

   int index = sentence.Integer( 2 );

   switch( index )
   {
      case 1:

         FixMode = FixUnavailable;
         break;

      case 2:

         FixMode = TwoDimensional;
         break;

      case 3:

         FixMode = ThreeDimensional;
         break;

      default:

         FixMode = FixUnknown;
         break;
   }

   index = 0;

   while( index < 12 )
   {
      SatelliteNumber[ index ] = sentence.Integer( index + 3 );
      index++;
   }

   PDOP = sentence.Double( 15 );
   HDOP = sentence.Double( 16 );
   VDOP = sentence.Double( 17 );

   return( TRUE );
}

BOOL GSA::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   switch( OperatingMode )
   {
      case Manual:

         sentence += "M";
         break;

      case Automatic:

         sentence += "A";
         break;

      default:

         sentence += "";
         break;
   }

   switch( FixMode )
   {
      case FixUnavailable:

         sentence += "1";
         break;

      case TwoDimensional:

         sentence += "2";
         break;

      case ThreeDimensional:

         sentence += "3";
         break;

      default:

         sentence += "";
         break;
   }

   int index = 0;

   while( index < 12 )
   {
      if ( SatelliteNumber[ index ] != 0 )
      {
         sentence += SatelliteNumber[ index ];
      }
      else
      {
         sentence += "";
      }

      index++;
   }

   sentence += PDOP;
   sentence += HDOP;
   sentence += VDOP;

   sentence.Finish();

   return( TRUE );
}

const GSA& GSA::operator = ( const GSA& source )
{
   OperatingMode = source.OperatingMode;
   FixMode       = source.FixMode;

   int index = 0;

   while( index < 12 )
   {
      SatelliteNumber[ index ] = source.SatelliteNumber[ index ];
      index++;
   }

   PDOP = source.PDOP;
   HDOP = source.HDOP;
   VDOP = source.VDOP;

   return( *this );
}
