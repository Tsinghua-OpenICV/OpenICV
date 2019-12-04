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
** $Workfile: hdt.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 3:01p $
*/



HDT::HDT()
{
   Mnemonic = "HDT";
   Empty();
}

HDT::~HDT()
{
   //Mnemonic.Empty();
   Empty();
}

void HDT::Empty( void )
{
   DegreesTrue = 0.0;
}

BOOL HDT::Parse( const SENTENCE& sentence )
{
   /*
   ** HDT - Heading - True
   **
   **        1   2 3
   **        |   | |
   ** $--HDT,x.x,T*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Heading Degrees, TRUE
   **  2) T = True
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

   DegreesTrue = sentence.Double( 1 );

   return( TRUE );
}

QString HDT::PlainEnglish( void ) const
{
   QString return_string;

   //return_string.Format( "Your true heading is %6.2lf degrees.", DegreesTrue );
   return_string = "Your true heading is " + QString::number(DegreesTrue) + "degrees.";
   return( return_string );
}

BOOL HDT::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += DegreesTrue;
   sentence += "T";

   sentence.Finish();

   return( TRUE );
}

const HDT& HDT::operator = ( const HDT& source )
{
   DegreesTrue = source.DegreesTrue;

   return( *this );
}
