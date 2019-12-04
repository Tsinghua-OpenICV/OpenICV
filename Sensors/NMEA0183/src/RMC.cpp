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
** $Workfile: rmc.cpp $
** $Revision: 5 $
** $Modtime: 10/12/98 6:43a $
*/



RMC::RMC()
{
  Mnemonic = "RMC";
  Empty();
}

RMC::~RMC()
{
  //Mnemonic.Empty();
  Empty();
}

void RMC::Empty( void )
{
  //UTCTime.Empty();
  IsDataValid                = NMEA_Unknown;
  SpeedOverGroundKnots       = 0.0;
  Position.Empty();
  TrackMadeGoodDegreesTrue   = 0.0;
  //Date.Empty();
  MagneticVariation          = 0.0;
  MagneticVariationDirection = EW_Unknown;
}

BOOL RMC::Parse( const SENTENCE& sentence )
{
/*
** RMC - Recommended Minimum Navigation Information
**                                                            12
**        1         2 3       4 5        6 7   8   9    10  11| 13
**        |         | |       | |        | |   |   |    |   | | |
** $--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,xxxx,x.x,a,a*hh<CR><LF>
**
** Field Number: 
**  1) UTC Time
**  2) Status, V = Navigation receiver warning
**  3) Latitude
**  4) N or S
**  5) Longitude
**  6) E or W
**  7) Speed over ground, knots
**  8) Track made good, degrees true
**  9) Date, ddmmyy
** 10) Magnetic Variation, degrees
** 11) E or W
** 12) Mode indication (special AG132) 
** 13) Checksum
  */
  
  /*
  ** First we check the checksum...
  */
  
  // looking for the checksum automatically if param is 0
  NMEA0183_BOOLEAN check = sentence.IsChecksumBad( 0 );
  
  if ( check == True )
  {
    SetErrorMessage( "Invalid Checksum" );
    return( FALSE );
  }
  
  if ( check == NMEA_Unknown )
  {
    SetErrorMessage( "Missing Checksum" );
    return( FALSE );
  } 
  
  UTCTime                    = sentence.Field( 1 );
  Time                       = sentence.Time( 1 );
  IsDataValid                = sentence.Boolean( 2 );
  Position.Parse( 3, 4, 5, 6, sentence );
  SpeedOverGroundKnots       = sentence.Double( 7 );
  TrackMadeGoodDegreesTrue   = sentence.Double( 8 );
  Date                       = sentence.Field( 9 );
  MagneticVariation          = sentence.Double( 10 );
  MagneticVariationDirection = sentence.EastOrWest( 11 );
  if (sentence.GetNumberOfDataFields() == 12)
    ModeIndication           = sentence.Field(12);
  
  return( TRUE );
}

QString RMC::PlainEnglish( void ) const
{
  QString return_string;
  
  //return_string.Empty();
  
  return_string = "At ";
  //return_string += Time.Format( "%I:%M.%S %p" );
  return_string = Time.time().toString();
  return_string += " UTC, you were at ";
  return_string += Position.PlainEnglish();
  return_string += ", making ";
  
  QString temp_string;
  
  //temp_string.Format( "%lf", SpeedOverGroundKnots );
  temp_string = QString::number(SpeedOverGroundKnots);
  
  
  return_string += temp_string;
  return_string += " knots, track made good ";
  
  //temp_string.Format( "%lf", TrackMadeGoodDegreesTrue );
  temp_string = QString::number( TrackMadeGoodDegreesTrue );
  return_string += temp_string;
  return_string += " degrees true.";
  
  return( return_string );
}

BOOL RMC::Write( SENTENCE& sentence )
{
/*
** Let the parent do its thing
  */
  
  RESPONSE::Write( sentence );
  
  sentence += UTCTime;
  sentence += IsDataValid;
  sentence += Position;
  sentence += SpeedOverGroundKnots;
  sentence += TrackMadeGoodDegreesTrue;
  sentence += Date;
  sentence += MagneticVariation;
  sentence += MagneticVariationDirection;
  
  sentence.Finish();
  
  return( TRUE );
}

const RMC& RMC::operator = ( const RMC& source )
{
  UTCTime                    = source.UTCTime;
  Time                       = source.Time;
  IsDataValid                = source.IsDataValid;
  Position                   = source.Position;
  SpeedOverGroundKnots       = source.SpeedOverGroundKnots;
  TrackMadeGoodDegreesTrue   = source.TrackMadeGoodDegreesTrue;
  Date                       = source.Date;
  MagneticVariation          = source.MagneticVariation;
  MagneticVariationDirection = source.MagneticVariationDirection;
  
  return( *this );
}
