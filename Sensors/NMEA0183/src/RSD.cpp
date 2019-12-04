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
** $Workfile: rsd.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:43p $
*/



RSD::RSD()
{
   Mnemonic = "RSD";
   Empty();
}

RSD::~RSD()
{
   //Mnemonic.Empty();
   Empty();
}

void RSD::Empty( void )
{
   Data1.Empty();
   Data2.Empty();
   CursorRangeFromOwnShip                = 0.0;
   CursorBearingDegreesClockwiseFromZero = 0.0;
   RangeScale                            = 0.0;
   //RangeUnits.Empty();
   DisplayRotation                       = RotationUnknown;
}

BOOL RSD::Parse( const SENTENCE& sentence )
{
   /*
   ** RSD - RADAR System Data
   **                                                        14
   **        1   2   3   4   5   6   7   8   9   10  11 12 13|
   **        |   |   |   |   |   |   |   |   |   |   |   | | |
   ** $--RSD,x.x,x.x,x.x,x.x,x.x,x.x,x.x,x.x,x.x,x.x,x.x,a,a*hh<CR><LF>
   **
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

   Data1.Parse( 1, sentence );
   Data2.Parse( 5, sentence );
   CursorRangeFromOwnShip                = sentence.Double( 9 );
   CursorBearingDegreesClockwiseFromZero = sentence.Double( 10 );
   RangeScale                            = sentence.Double( 11 );
   RangeUnits                            = sentence.Field( 12 );

   int temp_integer = sentence.Integer( 13 );

   switch( temp_integer )
   {
      case 'C':

         DisplayRotation = CourseUpRotation;
         break;

      case 'H':

         DisplayRotation = HeadUpRotation;
         break;

      case 'N':

         DisplayRotation = NorthUpRotation;
         break;

      default:

         DisplayRotation = RotationUnknown;
         break;
   }

   return( TRUE );
}

BOOL RSD::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   Data1.Write( sentence );
   Data2.Write( sentence );
   sentence += CursorRangeFromOwnShip;
   sentence += CursorBearingDegreesClockwiseFromZero;
   sentence += RangeScale;
   sentence += RangeUnits;

   switch( DisplayRotation )
   {
      case CourseUpRotation:

         sentence += "C";
         break;

      case HeadUpRotation:

         sentence += "H";
         break;

      case NorthUpRotation:

         sentence += "N";
         break;

      default:

         sentence += "";
         break;
   }

   sentence.Finish();

   return( TRUE );
}

const RSD& RSD::operator = ( const RSD& source )
{
   Data1                                 = source.Data1;
   Data2                                 = source.Data2;
   CursorRangeFromOwnShip                = source.CursorRangeFromOwnShip;
   CursorBearingDegreesClockwiseFromZero = source.CursorBearingDegreesClockwiseFromZero;
   RangeScale                            = source.RangeScale;
   RangeUnits                            = source.RangeUnits;
   DisplayRotation                       = source.DisplayRotation;

   return( *this );
}
