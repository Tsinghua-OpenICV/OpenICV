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
** $Workfile: sfi.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:37p $
*/



SFI::SFI()
{
   Mnemonic = "SFI";
   Empty();
}

SFI::~SFI()
{
   //Mnemonic.Empty();
   Empty();
}

void SFI::Empty( void )
{
   TotalMessages = 0.0;
   MessageNumber = 0.0;

   int index = 0;

   while( index < 6 )
   {
      Frequencies[ index ].Empty();
      index++;
   }

}

BOOL SFI::Parse( const SENTENCE& sentence )
{
   /*
   ** SFI - Scanning Frequency Information
   **
   **        1   2   3      4                     x
   **        |   |   |      |                     |
   ** $--SFI,x.x,x.x,xxxxxx,c .......... xxxxxx,c*hh<CR><LF>
   **
   **  1) Total Number Of Messages
   **  2) Message Number
   **  3) Frequency 1
   **  4) Mode 1
   **  x) Checksum
   */

   /*
   ** First we check the checksum...
   */

   int number_of_data_fields = sentence.GetNumberOfDataFields();

   if ( sentence.IsChecksumBad( number_of_data_fields + 1 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   TotalMessages = sentence.Double( 1 );
   MessageNumber = sentence.Double( 2 );

   /*
   ** Clear out any old data
   */

   int index = 0;

   while( index < 6 )
   {
      Frequencies[ index ].Empty();
      index++;
   }

   int number_of_frequencies = ( number_of_data_fields - 2 ) / 2;
   int frequency_number      = 0;

   /*
   ** index is the number of data fields before the frequency/mode +
   ** the frequency number times the number of fields in a FREQUENC_AND_MODE
   */

   while( frequency_number < number_of_frequencies )
   {
      index = 2 + ( frequency_number * 2 );

      Frequencies[ frequency_number ].Parse( index, sentence );

      frequency_number++;
   }

   return( TRUE );
}

BOOL SFI::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += TotalMessages;
   sentence += MessageNumber;

   int index = 0;

   while( index < 6 )
   {
      Frequencies[ index ].Write( sentence );
      index++;
   }

   sentence.Finish();

   return( TRUE );
}

const SFI& SFI::operator = ( const SFI& source )
{
   TotalMessages = source.TotalMessages;
   MessageNumber = source.MessageNumber;

   int index = 0;

   while( index < 6 )
   {
      Frequencies[ index ] = source.Frequencies[ index ];
      index++;
   }

   return( *this );
}
