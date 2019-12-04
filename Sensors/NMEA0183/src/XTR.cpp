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
** $Workfile: xtr.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:40p $
*/



XTR::XTR()
{
   Mnemonic = "XTR";
   Empty();
}

XTR::~XTR()
{
   //Mnemonic.Empty();
   Empty();
}

void XTR::Empty( void )
{
   Magnitude        = 0.0;
   DirectionToSteer = LR_Unknown;
}

BOOL XTR::Parse( const SENTENCE& sentence )
{
   /*
   ** XTR - Cross Track Error - Dead Reckoning
   **
   **        1   2 3 4
   **        |   | | |
   ** $--XTR,x.x,a,N*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Magnitude of cross track error
   **  2) Direction to steer, L or R
   **  3) Units, N = Nautical Miles
   **  4) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 4 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   Magnitude        = sentence.Double( 1 );
   DirectionToSteer = sentence.LeftOrRight( 2 );

   return( TRUE );
}

BOOL XTR::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += Magnitude;
   sentence += DirectionToSteer;
   sentence += "N";

   sentence.Finish();

   return( TRUE );
}

const XTR& XTR::operator = ( const XTR& source )
{
   Magnitude        = source.Magnitude;
   DirectionToSteer = source.DirectionToSteer;

   return( *this );
}
