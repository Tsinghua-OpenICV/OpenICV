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
** $Workfile: xdr.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:20p $
*/



XDR::XDR()
{
   Mnemonic = "XDR";
   Empty();
}

XDR::~XDR()
{
   //Mnemonic.Empty();
   Empty();
}

void XDR::Empty( void )
{
   delete_all_entries();
}

BOOL XDR::Parse( const SENTENCE& sentence )
{
   /*
   ** XDR - Cross Track Error - Dead Reckoning
   **
   **        1 2   3 4			    n
   **        | |   | |            |
   ** $--XDR,a,x.x,a,c--c, ..... *hh<CR><LF>
   **
   ** Field Number: 
   **  1) Transducer Type
   **  2) Measurement Data
   **  3) Units of measurement
   **  4) Name of transducer
   **  x) More of the same
   **  n) Checksum
   */

   delete_all_entries();

   int field_number = 1;

   TRANSDUCER_DATA *transducer_data_p = (TRANSDUCER_DATA *) NULL;

   while( sentence.Field( field_number + 1 ) != "" )
   {
      transducer_data_p = new TRANSDUCER_DATA;

      transducer_data_p->Parse( field_number, sentence );
      //Transducers.Add( transducer_data_p );
	  Transducers.resize(Transducers.size()+1);
      Transducers.insert( Transducers.size(),transducer_data_p );
      
      field_number += 4;
   }

   return( TRUE );
}

BOOL XDR::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   int index = 0;
   int number_of_entries = Transducers.size();

   while( index < number_of_entries )
   {
      ( (TRANSDUCER_DATA *) Transducers[ index ] )->Write( sentence );
      index++;
   }

   sentence.Finish();

   return( TRUE );
}

void XDR::delete_all_entries( void )
{
   int loop_index = 0;
   int number_of_entries = Transducers.size();

   TRANSDUCER_DATA * data_p = (TRANSDUCER_DATA *) NULL;

   /*while( loop_index < number_of_entries )
   {
      data_p = (TRANSDUCER_DATA *) Transducers[ loop_index ];
      Transducers[ loop_index ] = NULL;
      delete data_p;
      loop_index++;
   }*/

   Transducers.clear();
}

TRANSDUCER_DATA::TRANSDUCER_DATA()
{
   Empty();
}

TRANSDUCER_DATA::~TRANSDUCER_DATA()
{
   Empty();
}

void TRANSDUCER_DATA::Empty( void )
{
   TransducerType  = TransducerUnknown;
   MeasurementData = 0.0;
   //MeasurementUnits.Empty();
   //TransducerName.Empty();
}

void TRANSDUCER_DATA::Parse( int first_field_number, const SENTENCE& sentence )
{
   TransducerType   = sentence.TransducerType( first_field_number );
   MeasurementData  = sentence.Double( first_field_number + 1 );
   MeasurementUnits = sentence.Field(  first_field_number + 2 );
   TransducerName   = sentence.Field(  first_field_number + 3 );
}

void TRANSDUCER_DATA::Write( SENTENCE& sentence )
{
   sentence += TransducerType;
   sentence += MeasurementData;
   sentence += MeasurementUnits;
   sentence += TransducerName;
}

const TRANSDUCER_DATA& TRANSDUCER_DATA::operator = ( const TRANSDUCER_DATA& source )
{
   TransducerType   = source.TransducerType;
   MeasurementData  = source.MeasurementData;
   MeasurementUnits = source.MeasurementUnits;
   TransducerName   = source.TransducerName;

   return( *this );
}
