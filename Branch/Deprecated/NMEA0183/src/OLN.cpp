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
** $Workfile: oln.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:42p $
*/


OLN::OLN()
{
   Mnemonic = "OLN";
   Empty();
}

OLN::~OLN()
{
   //Mnemonic.Empty();
   Empty();
}

void OLN::Empty( void )
{
   Pair1.Empty();
   Pair2.Empty();
   Pair3.Empty();
}

BOOL OLN::Parse( const SENTENCE& sentence )
{
   /*
   ** OLN - Omega Lane Numbers
   **
   **        1          2          3          4
   **        |--------+ |--------+ |--------+ |
   ** $--OLN,aa,xxx,xxx,aa,xxx,xxx,aa,xxx,xxx*hh<CR><LF>
   **
   **  1) Omega Pair 1
   **  2) Omega Pair 1
   **  3) Omega Pair 1
   **  4) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 10 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   Pair1.Parse( 1, sentence );
   Pair2.Parse( 4, sentence );
   Pair3.Parse( 7, sentence );

   return( TRUE );
}

BOOL OLN::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   Pair1.Write( sentence );
   Pair2.Write( sentence );
   Pair3.Write( sentence );

   sentence.Finish();

   return( TRUE );
}

const OLN& OLN::operator = ( const OLN& source )
{
   Pair1 = source.Pair1;
   Pair2 = source.Pair2;
   Pair3 = source.Pair3;

   return( *this );
}
