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
** $Workfile: zda.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:40p $
*/



ZDA::ZDA()
{
   Mnemonic = "ZDA";
   Empty();
}

ZDA::~ZDA()
{
   //Mnemonic.Empty();
   Empty();
}

void ZDA::Empty( void )
{
   //UTCTime.Empty();
   Day                   = 0;
   Month                 = 0;
   Year                  = 0;
   LocalHourDeviation    = 0;
   LocalMinutesDeviation = 0;
}

BOOL ZDA::Parse( const SENTENCE& sentence )
{
   /*
   ** ZDA - Time & Date
   ** UTC, day, month, year and local time zone
   **
   ** $--ZDA,hhmmss.ss,xx,xx,xxxx,xx,xx*hh<CR><LF>
   **        |         |  |  |    |  |
   **        |         |  |  |    |  +- Local zone minutes description, same sign as local hours
   **        |         |  |  |    +- Local zone description, 00 to +- 13 hours
   **        |         |  |  +- Year
   **        |         |  +- Month, 01 to 12
   **        |         +- Day, 01 to 31
   **        +- Universal Time Coordinated (UTC)
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 7 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   UTCTime = sentence.Field( 1 );

   char temp_number[ 3 ];

   temp_number[ 2 ] = 0x00;

   temp_number[ 0 ] = UTCTime[ 0 ].toLatin1();
   temp_number[ 1 ] = UTCTime[ 1 ].toLatin1();

   int hours = ::atoi( temp_number );

   temp_number[ 0 ] = UTCTime[ 2 ].toLatin1();
   temp_number[ 1 ] = UTCTime[ 3 ].toLatin1();

   int minutes = ::atoi( temp_number );

   temp_number[ 0 ] = UTCTime[ 4 ].toLatin1();
   temp_number[ 1 ] = UTCTime[ 5 ].toLatin1();

   int seconds = ::atoi( temp_number );

   Day                   = sentence.Integer( 2 );
   Month                 = sentence.Integer( 3 );
   Year                  = sentence.Integer( 4 );
   LocalHourDeviation    = sentence.Integer( 5 );
   LocalMinutesDeviation = sentence.Integer( 6 );

   //Time = CTime( Year, Month, Day, hours, minutes, seconds );
   Time = QDateTime(QDate(Year, Month, Day) , QTime(hours, minutes, seconds) );

   return( TRUE );
}

BOOL ZDA::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += UTCTime;
   sentence += Day;
   sentence += Month;
   sentence += Year;
   sentence += LocalHourDeviation;
   sentence += LocalMinutesDeviation;

   sentence.Finish();

   return( TRUE );
}

const ZDA& ZDA::operator = ( const ZDA& source )
{
   UTCTime               = source.UTCTime;
   Time                  = source.Time;
   Day                   = source.Day;
   Month                 = source.Month;
   Year                  = source.Year;
   LocalHourDeviation    = source.LocalHourDeviation;
   LocalMinutesDeviation = source.LocalMinutesDeviation;

   return( *this );
}
