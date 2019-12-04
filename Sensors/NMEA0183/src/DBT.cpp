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
** $Workfile: dbt.cpp $
** $Revision: 5 $
** $Modtime: 10/10/98 2:45p $
*/


DBT::DBT()
{
   Mnemonic = "DBT";
   Empty();
}

DBT::~DBT()
{
   //Mnemonic.Empty();
   Empty();
}

void DBT::Empty( void )
{
   DepthFeet    = 0.0;
   DepthMeters  = 0.0;
   DepthFathoms = 0.0;
}

BOOL DBT::Parse( const SENTENCE& sentence )
{
   /*
   ** DBT - Depth below transducer
   **
   **        1   2 3   4 5   6 7
   **        |   | |   | |   | |
   ** $--DBT,x.x,f,x.x,M,x.x,F*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Depth, feet
   **  2) f = feet
   **  3) Depth, meters
   **  4) M = meters
   **  5) Depth, Fathoms
   **  6) F = Fathoms
   **  7) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 7 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   DepthFeet    = sentence.Double( 1 );
   DepthMeters  = sentence.Double( 3 );
   DepthFathoms = sentence.Double( 5 );

   return( TRUE );
}

BOOL DBT::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += DepthFeet;
   sentence += "f";
   sentence += DepthMeters;
   sentence += "M";
   sentence += DepthFathoms;
   sentence += "F";

   sentence.Finish();

   return( TRUE );
}

const DBT& DBT::operator = ( const DBT& source )
{
   DepthFeet    = source.DepthFeet;
   DepthMeters  = source.DepthMeters;
   DepthFathoms = source.DepthFathoms;

   return( *this );
}
