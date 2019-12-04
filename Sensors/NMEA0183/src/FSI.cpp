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
** $Workfile: fsi.cpp $
** $Revision: 5 $
** $Modtime: 10/10/98 2:46p $
*/



FSI::FSI()
{
   Mnemonic = "FSI";
   Empty();
}

FSI::~FSI()
{
   //Mnemonic.Empty();
   Empty();
}

void FSI::Empty( void )
{
   TransmittingFrequency = 0.0;
   ReceivingFrequency    = 0.0;
   Mode                  = CommunicationsModeUnknown;
   PowerLevel            = 0;
}

BOOL FSI::Parse( const SENTENCE& sentence )
{
   /*
   ** FSI - Frequency Set Information
   **
   **        1      2      3 4 5
   **        |      |      | | |
   ** $--FSI,xxxxxx,xxxxxx,c,x*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Transmitting Frequency
   **  2) Receiving Frequency
   **  3) Communications Mode
   **  4) Power Level
   **  5) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 5 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   TransmittingFrequency = sentence.Double( 1 );
   ReceivingFrequency    = sentence.Double( 2 );
   Mode                  = sentence.CommunicationsMode( 3 );
   PowerLevel            = (short) sentence.Integer( 4 );

   return( TRUE );
}

BOOL FSI::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += TransmittingFrequency;
   sentence += ReceivingFrequency;
   sentence += Mode;
   sentence += PowerLevel;

   sentence.Finish();

   return( TRUE );
}

const FSI& FSI::operator = ( const FSI& source )
{
   TransmittingFrequency = source.TransmittingFrequency;
   ReceivingFrequency    = source.ReceivingFrequency;
   Mode                  = source.Mode;
   PowerLevel            = source.PowerLevel;

   return( *this );
}
