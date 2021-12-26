#include "nmea0183.h"
#pragma hdrstop


GST::GST()
{
   Mnemonic = "GST";
   Empty();
}

GST::~GST()
{
   //Mnemonic.Empty();
   Empty();
}

void GST::Empty( void )
{
   //UTCTime.Empty();
	RMSvalue = 0.0;
	ErrorEllipseMajor = 0.0;
	ErrorEllipseMinor = 0.0;
	ErrorEllipseOrientation = 0.0;
	LatitudeError = 0.0;
	LongitudeError = 0.0;
	HeightError = 0.0;
}

BOOL GST::Parse( const SENTENCE& sentence )
{
   /*
   ** GST - Global Positioning System Sentence Translator
   ** Position error statistics.
   **
   **                                                      
   **        1        2     3     4     5     6     7     8      9  
   **        |        |     |     |     |     |     |     |      |   
   ** $--GST,hhmmss.s,x.xxx,x.xxx,x.xxx,xxx.x,x.xxx,x.xxx,x.xxx*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Universal Time Coordinated (UTC)
   **  2) RMS value of the pseudorange residuals
   **  3) Error ellipse semi-major axis 1 sigma error, in meters
   **  4) Error ellipse semi-minor axis 1 sigma error, in meters
   **  5) Error ellipse orientation, degrees from true north
   **  6) Latitude 1 sigma error, in meters
   **  7) Longitude 1 sigma error, in meters
   **  8) Height 1 sigma error, in meters
   **  9) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 15 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

	UTCTime        = sentence.Field( 1 );
  Time           = sentence.Time( 1 );
	RMSvalue				= sentence.Double( 2 );
	ErrorEllipseMajor		= sentence.Double( 3 );
	ErrorEllipseMinor		= sentence.Double( 4 );
	ErrorEllipseOrientation = sentence.Double( 5 );
	LatitudeError			= sentence.Double( 6 );
	LongitudeError			= sentence.Double( 7 );
	HeightError				= sentence.Double( 8 );
    return( TRUE );
}


BOOL GST::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += UTCTime;
   sentence += RMSvalue;
   sentence += ErrorEllipseMajor;
   sentence += "M";
   sentence += ErrorEllipseMinor;
   sentence += "M";
   sentence += ErrorEllipseOrientation;
   sentence += LatitudeError;
   sentence += "M";
   sentence += LongitudeError;
   sentence += "M";
   sentence += HeightError;
   sentence += "M";
   sentence.Finish();
   
   return( TRUE );
}

const GST& GST::operator = ( const GST& source )
{
  UTCTime                 = source.UTCTime;
  Time           = source.Time; 
  RMSvalue				= source.RMSvalue;
  ErrorEllipseMajor		= source.ErrorEllipseMajor;
  ErrorEllipseMinor		= source.ErrorEllipseMinor;
  ErrorEllipseOrientation = source.ErrorEllipseOrientation;
  LatitudeError			= source.LatitudeError;
  LongitudeError			= source.LongitudeError;
  HeightError				= source.HeightError;
  
  return( *this );
}
