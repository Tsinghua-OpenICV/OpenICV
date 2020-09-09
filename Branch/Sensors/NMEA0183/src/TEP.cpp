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
** $Workfile: tep.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:56p $
*/

/*
** This Sentence Not Recommended For New Designs
** There is no recommended replacement.
*/


TEP::TEP()
{
   Mnemonic = "TEP";
   Empty();
}

TEP::~TEP()
{
   //Mnemonic.Empty();
   Empty();
}

void TEP::Empty( void )
{
   ElevationDegrees = 0.0;
}

BOOL TEP::Parse( const SENTENCE& sentence )
{
   /*
   ** TEP - TRANSIT Satellite Predicted Elevation
   **
   **        1   2 3
   **        |   | |
   ** $--TEP,x.x,T*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Elevation degrees
   **  2) D = Degrees
   **  3) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 3 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   ElevationDegrees = sentence.Double( 1 );

   return( TRUE );
}

QString TEP::PlainEnglish( void ) const
{
   QString return_string;

   //return_string.Format( "TRANSIT satellite predicted elevation is %6.2lf degrees.", ElevationDegrees );
   return_string = "TRANSIT satellite predicted elevation is" + QString::number(ElevationDegrees ) + "degrees."  ;


   return( return_string );
}

BOOL TEP::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += ElevationDegrees;
   sentence += "D";

   sentence.Finish();

   return( TRUE );
}

const TEP& TEP::operator = ( const TEP& source )
{
   ElevationDegrees = source.ElevationDegrees;

   return( *this );
}
