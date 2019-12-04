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
** $Workfile: zfi.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:43p $
*/

/*
** This Sentence Not Recommended For New Designs
** ZFO is recommended.
*/



ZFI::ZFI()
{
   Mnemonic = "ZFI";
   Empty();
}

ZFI::~ZFI()
{
   //Mnemonic.Empty();
   Empty();
}

void ZFI::Empty( void )
{
   //UTCTimeString.Empty();
   //ElapsedTimeString.Empty();
   //From.Empty();
}

BOOL ZFI::Parse( const SENTENCE& sentence )
{
   /*
   ** ZFI - Elapsed time from point of interest
   **
   **        1         2         3    4
   **        |         |         |    |  
   ** $--ZFI,hhmmss.ss,hhmmss.ss,c--c*hh<CR><LF>
   **
   ** 1) Universal Time Coordinated (UTC)
   ** 2) Elapsed Time
   ** 3) Waypoint ID (From)
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

   ElapsedTimeString = sentence.Field( 2 );

   temp_number[ 0 ] = ElapsedTimeString[ 0 ].toLatin1();
   temp_number[ 1 ] = ElapsedTimeString[ 1 ].toLatin1();

   hours = ::atoi( temp_number );

   temp_number[ 0 ] = ElapsedTimeString[ 2 ].toLatin1();
   temp_number[ 1 ] = ElapsedTimeString[ 3 ].toLatin1();

   minutes = ::atoi( temp_number );

   temp_number[ 0 ] = ElapsedTimeString[ 4 ].toLatin1();
   temp_number[ 1 ] = ElapsedTimeString[ 5 ].toLatin1();

   seconds = ::atoi( temp_number );

   //ElapsedTime = CTime( year, month, day, hours, minutes, seconds );
   ElapsedTime = QDateTime(QDate(year, month, day) , QTime(hours, minutes, seconds) );

   From = sentence.Field( 3 );

   return( TRUE );
}

BOOL ZFI::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += UTCTimeString;
   sentence += ElapsedTimeString;
   sentence += From;

   sentence.Finish();

   return( TRUE );
}

const ZFI& ZFI::operator = ( const ZFI& source )
{
   UTCTimeString     = source.UTCTimeString;
   UTCTime           = source.UTCTime;
   ElapsedTimeString = source.ElapsedTimeString;
   ElapsedTime       = source.ElapsedTime;
   From              = source.From;

   return( *this );
}
