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
** $Workfile: zzu.cpp $
** $Revision: 5 $
** $Modtime: 10/10/98 2:38p $
*/

/*
** This Sentence Not Recommended For New Designs
** ZDA is recommended.
*/



ZZU::ZZU()
{
   Mnemonic = "ZZU";
   Empty();
}

ZZU::~ZZU()
{
   //Mnemonic.Empty();
   Empty();
}

void ZZU::Empty( void )
{
   //UTCTimeString.Empty();
}

BOOL ZZU::Parse( const SENTENCE& sentence )
{
   /*
   ** ZZU - Time UTC
   **
   **        1          2
   **        |          |  
   ** $--ZZU,hhmmss.ss,*hh<CR><LF>
   **
   ** 1) Universal Time Coordinated (UTC)
   ** 2) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 2 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   }

   QDateTime temp_time = QDateTime::currentDateTime();

   int year  = temp_time.date().year();
   int month = temp_time.date().month();
   int day   = temp_time.date().day();


   UTCTimeString = sentence.Field( 1 );

   char temp_number[ 3 ];

   temp_number[ 2 ] = 0x00;

   temp_number[ 0 ] = UTCTimeString[ 0 ].toLatin1();
   temp_number[ 1 ] = UTCTimeString[ 1 ].toLatin1();

   int hours = ::atoi( temp_number );

   temp_number[ 0 ] = UTCTimeString[ 2 ].toLatin1();
   temp_number[ 1 ] = UTCTimeString[ 3 ].toLatin1();

   int minutes = ::atoi( temp_number );

   temp_number[ 0 ] = UTCTimeString[ 4 ].toLatin1();
   temp_number[ 1 ] = UTCTimeString[ 5 ].toLatin1();

   int seconds = ::atoi( temp_number );

   //UTCTime = CTime( year, month, day, hours, minutes, seconds );
   UTCTime = QDateTime(QDate(year, month, day) , QTime(hours, minutes, seconds) );

   return( TRUE );
}

BOOL ZZU::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += UTCTimeString;

   sentence.Finish();

   return( TRUE );
}

const ZZU& ZZU::operator = ( const ZZU& source )
{
   UTCTimeString      = source.UTCTimeString;
   UTCTime            = source.UTCTime;

   return( *this );
}
