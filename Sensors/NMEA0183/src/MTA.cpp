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
** $Workfile: mta.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:56p $
*/

/*
** This Sentence Not Recommended For New Designs
** XDR is recommended.
*/



MTA::MTA()
{
   Mnemonic = "MTA";
   Empty();
}

MTA::~MTA()
{
   //Mnemonic.Empty();
   Empty();
}

void MTA::Empty( void )
{
   Temperature = 0.0;
   //UnitOfMeasurement.Empty();
}

BOOL MTA::Parse( const SENTENCE& sentence )
{
   /*
   ** MTA - Air Temperature
   **
   **        1   2 3
   **        |   | | 
   ** $--MTA,x.x,C*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Degrees
   **  2) Unit of Measurement, Celcius
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

   Temperature       = sentence.Double( 1 );
   UnitOfMeasurement = sentence.Field( 2 );

   return( TRUE );
}

QString MTA::PlainEnglish( void ) const
{
   QString return_string;

   //return_string.Empty();

   //return_string.Format( "The air temperature is %3.1lf ", Temperature ); 
   return_string = "The air temperature is" + QString::number( Temperature );
   return_string += UnitOfMeasurement;
   return_string += ".";

   return( return_string );
}

BOOL MTA::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += Temperature;
   sentence += UnitOfMeasurement;

   sentence.Finish();

   return( TRUE );
}

const MTA& MTA::operator = ( const MTA& source )
{
   Temperature       = source.Temperature;
   UnitOfMeasurement = source.UnitOfMeasurement;

   return( *this );
}
