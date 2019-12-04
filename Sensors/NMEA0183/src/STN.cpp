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
** $Workfile: stn.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:40p $
*/


STN::STN()
{
   Mnemonic = "STN";
   Empty();
}

STN::~STN()
{
   //Mnemonic.Empty();
   Empty();
}

void STN::Empty( void )
{
   TalkerIDNumber = 0;
}

BOOL STN::Parse( const SENTENCE& sentence )
{
   /*
   ** STN - Multiple Data ID
   **
   **        1   2
   **        |   |
   ** $--STN,x.x,*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Talker ID Number
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

   TalkerIDNumber = sentence.Integer( 1 );

   return( TRUE );
}

BOOL STN::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += TalkerIDNumber;

   sentence.Finish();

   return( TRUE );
}

const STN& STN::operator = ( const STN& source )
{
   TalkerIDNumber = source.TalkerIDNumber;

   return( *this );
}
