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
** $Workfile: zlz.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:39p $
*/

/*
** This Sentence Not Recommended For New Designs
** ZDA is recommended.
*/


ZLZ::ZLZ()
{
   Mnemonic = "ZLZ";
   Empty();
}

ZLZ::~ZLZ()
{
   //Mnemonic.Empty();
   Empty();
}

void ZLZ::Empty( void )
{
   //UTCTimeString.Empty();
   //LocalTimeString.Empty();
   LocalHourDeviation = 0;
}

BOOL ZLZ::Parse( const SENTENCE& sentence )
{
   /*
   ** ZLZ - Time of Day
   ** UTC, local time, zone
   **
   **        1         2         3  4
   **        |         |         |  |  
   ** $--ZLZ,hhmmss.ss,hhmmss.ss,xx*hh<CR><LF>
   **
   ** 1) Universal Time Coordinated (UTC)
   ** 2) Local Time
   ** 3) Local Zone Description, number of whole hours added to local time to obtain GMT
   ** 4) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 4 ) == True )
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

   LocalTimeString = sentence.Field( 2 );

   temp_number[ 0 ] = LocalTimeString[ 0 ].toLatin1();
   temp_number[ 1 ] = LocalTimeString[ 1 ].toLatin1();

   hours = ::atoi( temp_number );

   temp_number[ 0 ] = LocalTimeString[ 2 ].toLatin1();
   temp_number[ 1 ] = LocalTimeString[ 3 ].toLatin1();

   minutes = ::atoi( temp_number );

   temp_number[ 0 ] = LocalTimeString[ 4 ].toLatin1();
   temp_number[ 1 ] = LocalTimeString[ 5 ].toLatin1();

   seconds = ::atoi( temp_number );

   //LocalTime = CTime( year, month, day, hours, minutes, seconds );
   LocalTime = QDateTime(QDate(year, month, day) , QTime(hours, minutes, seconds) );

   LocalHourDeviation = sentence.Integer( 3 );

   return( TRUE );
}

BOOL ZLZ::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += UTCTimeString;
   sentence += LocalTimeString;
   sentence += LocalHourDeviation;

   sentence.Finish();

   return( TRUE );
}

const ZLZ& ZLZ::operator = ( const ZLZ& source )
{
   UTCTimeString      = source.UTCTimeString;
   UTCTime            = source.UTCTime;
   LocalTimeString    = source.LocalTimeString;
   LocalTime          = source.LocalTime;
   LocalHourDeviation = source.LocalHourDeviation;

   return( *this );
}
