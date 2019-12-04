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
** $Workfile: mtw.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:56p $
*/



MTW::MTW()
{
   Mnemonic = "MTW";
   Empty();
}

MTW::~MTW()
{
   //Mnemonic.Empty();
   Empty();
}

void MTW::Empty( void )
{
   Temperature = 0.0;
   //UnitOfMeasurement.Empty();
}

BOOL MTW::Parse( const SENTENCE& sentence )
{
   /*
   ** MTW - Water Temperature
   **
   **        1   2 3
   **        |   | | 
   ** $--MTW,x.x,C*hh<CR><LF>
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

QString MTW::PlainEnglish( void ) const
{
   QString return_string;

   //return_string.Format( "The water temperature is %3.1lf ", Temperature );
   return_string = "The water temperature is" + QString::number( Temperature );

   return_string += UnitOfMeasurement;
   return_string += ".";

   return( return_string );
}

BOOL MTW::Write( SENTENCE& sentence )
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

const MTW& MTW::operator = ( const MTW& source )
{
   Temperature       = source.Temperature;
   UnitOfMeasurement = source.UnitOfMeasurement;

   return( *this );
}
