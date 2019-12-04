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
** $Workfile: hcc.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:55p $
*/

#ifdef _MSC_VER
#   pragma warning(disable:4996)
#endif // _MSC_VER

/*
** This Sentence Not Recommended For New Designs
** Use of HDG is recommended.
*/

HCC::HCC()
{
   Mnemonic = "HCC";
   Empty();
}

HCC::~HCC()
{
   //Mnemonic.Empty();
   Empty();
}

void HCC::Empty( void )
{
   HeadingDegrees = 0.0;
}

BOOL HCC::Parse( const SENTENCE& sentence )
{
   /*
   ** HCC - Compass Heading
   **       Vessel compass heading, which differs from magnetic heading by the amount of
   **       uncorrected magnetic deviation.
   **
   **        1   2
   **        |   |
   ** $--HCC,x.x*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Commpass heading, degrees
   **  2) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 2 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   HeadingDegrees = sentence.Double( 1 );

   return( TRUE );
}

QString HCC::PlainEnglish( void ) const
{
   QString return_string;

   return_string = "The compass heading is ";

   char temp_string[ 80 ];

   ::sprintf( temp_string, "%6.2lf degrees.", HeadingDegrees );

   return_string += temp_string;

   return( return_string );
}

BOOL HCC::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += HeadingDegrees;
   
   sentence.Finish();

   return( TRUE );
}

const HCC& HCC::operator = ( const HCC& source )
{
   HeadingDegrees = source.HeadingDegrees;

   return( *this );
}
