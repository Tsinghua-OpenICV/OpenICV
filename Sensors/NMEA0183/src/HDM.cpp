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
** $Workfile: hdm.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:55p $
*/

/*
** This Sentence Not Recommended For New Designs
** Use of HDG is recommended.
*/

#ifdef _MSC_VER
#   pragma warning(disable:4996)
#endif // _MSC_VER

HDM::HDM()
{
   Mnemonic = "HDM";
   Empty();
}

HDM::~HDM()
{
   //Mnemonic.Empty();
   Empty();
}

void HDM::Empty( void )
{
   HeadingDegrees = 0.0;
}

BOOL HDM::Parse( const SENTENCE& sentence )
{
   /*
   ** HDM - Heading - Deviation & Variation
   **
   **        1   2 3
   **        |   | |
   ** $--HDM,x.x,M*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Heading in degrees
   **  2) M, Magnetic Deviation
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

   HeadingDegrees = sentence.Double( 1 );

   return( TRUE );
}

QString HDM::PlainEnglish( void ) const
{
   QString return_string;

   return_string = "The Magnetic heading is ";

   char temp_string[ 80 ];

   ::sprintf( temp_string, "%6.2lf degrees.", HeadingDegrees );

   return_string += temp_string;

   return( return_string );
}

BOOL HDM::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += HeadingDegrees;
   sentence += "M";

   sentence.Finish();

   return( TRUE );
}

const HDM& HDM::operator = ( const HDM& source )
{
   HeadingDegrees = source.HeadingDegrees;

   return( *this );
}
