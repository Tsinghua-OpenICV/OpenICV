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
** $Workfile: vwe.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:39p $
*/

/*
** This Sentence Not Recommended For New Designs
** There is no recommended replacement.
*/



VWE::VWE()
{
   Mnemonic = "VWE";
   Empty();
}

VWE::~VWE()
{
   //Mnemonic.Empty();
   Empty();
}

void VWE::Empty( void )
{
   EfficiencyPercent = 0;
}

BOOL VWE::Parse( const SENTENCE& sentence )
{
   /*
   ** VWE - Wind Track Efficiency
   **
   **        1   2
   **        |   |
   ** $--VWE,x.x,*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Efficiency, Percent
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

   EfficiencyPercent = sentence.Integer( 1 );

   return( TRUE );
}

BOOL VWE::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += EfficiencyPercent;

   sentence.Finish();

   return( TRUE );
}

const VWE& VWE::operator = ( const VWE& source )
{
   EfficiencyPercent = source.EfficiencyPercent;

   return( *this );
}
