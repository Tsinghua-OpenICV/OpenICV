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
** $Workfile: zti.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 3:01p $
*/

/*
** This Sentence Not Recommended For New Designs
** ZTG is recommended.
*/



ZTI::ZTI()
{
   Mnemonic = "ZTI";
   Empty();
}

ZTI::~ZTI()
{
   //Mnemonic.Empty();
   Empty();
}

void ZTI::Empty( void )
{
   //UTCTimeString.Empty();
   //TimeToGoString.Empty();
   //To.Empty();
}

BOOL ZTI::Parse( const SENTENCE& sentence )
{
   /*
   ** ZTI - Estimated time to point of interest
   **
   **        1         2         3    4
   **        |         |         |    |  
   ** $--ZTI,hhmmss.ss,hhmmss.ss,c--c*hh<CR><LF>
   **
   ** 1) Universal Time Coordinated (UTC)
   ** 2) Arrival Time
   ** 3) Waypoint ID (To)
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

   TimeToGoString = sentence.Field( 2 );

   temp_number[ 0 ] = TimeToGoString[ 0 ].toLatin1();
   temp_number[ 1 ] = TimeToGoString[ 1 ].toLatin1();

   hours = ::atoi( temp_number );

   temp_number[ 0 ] = TimeToGoString[ 2 ].toLatin1();
   temp_number[ 1 ] = TimeToGoString[ 3 ].toLatin1();

   minutes = ::atoi( temp_number );

   temp_number[ 0 ] = TimeToGoString[ 4 ].toLatin1();
   temp_number[ 1 ] = TimeToGoString[ 5 ].toLatin1();

   seconds = ::atoi( temp_number );

   //TimeToGo = CTime( year, month, day, hours, minutes, seconds );
   TimeToGo = QDateTime(QDate(year, month, day) , QTime(hours, minutes, seconds) );

   To = sentence.Field( 3 );

   return( TRUE );
}

BOOL ZTI::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += UTCTimeString;
   sentence += TimeToGoString;
   sentence += To;

   sentence.Finish();

   return( TRUE );
}

const ZTI& ZTI::operator = ( const ZTI& source )
{
   UTCTimeString  = source.UTCTimeString;
   UTCTime        = source.UTCTime;
   TimeToGoString = source.TimeToGoString;
   TimeToGo       = source.TimeToGo;
   To             = source.To;

   return( *this );
}
