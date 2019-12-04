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
** $Workfile: alm.cpp $
** $Revision: 6 $
** $Modtime: 10/10/98 2:41p $
*/


ALM::ALM()
{
   Mnemonic = "ALM";
   Empty();
}

ALM::~ALM()
{
   //Mnemonic.Empty();
   Empty();
}

void ALM::Empty( void )
{
   NumberOfMessages         = 0;
   MessageNumber            = 0;
   PRNNumber                = 0;
   WeekNumber               = 0;
   SVHealth                 = 0;
   Eccentricity             = 0;
   AlmanacReferenceTime     = 0;
   InclinationAngle         = 0;
   RateOfRightAscension     = 0;
   RootOfSemiMajorAxis      = 0;
   ArgumentOfPerigee        = 0;
   LongitudeOfAscensionNode = 0;
   MeanAnomaly              = 0;
   F0ClockParameter         = 0;
   F1ClockParameter         = 0;
}

BOOL ALM::Parse( const SENTENCE& sentence )
{
   QString field_data;

   /*
   ** ALM - GPS Almanac Data
   **
   **        1   2   3  4   5  6    7  8    9    10     11     12     13     14  15   16
   **        |   |   |  |   |  |    |  |    |    |      |      |      |      |   |    |
   ** $--ALM,x.x,x.x,xx,x.x,hh,hhhh,hh,hhhh,hhhh,hhhhhh,hhhhhh,hhhhhh,hhhhhh,hhh,hhh,*hh<CR><LF>
   **
   **  1) Total number of messages
   **  2) Message Number
   **  3) Satellite PRN number (01 to 32)
   **  4) GPS Week Number
   **  5) SV health, bits 17-24 of each almanac page
   **  6) Eccentricity
   **  7) Almanac Reference Time
   **  8) Inclination Angle
   **  9) Rate of Right Ascension
   ** 10) Root of semi-major axis
   ** 11) Argument of perigee
   ** 12) Longitude of ascension node
   ** 13) Mean anomaly
   ** 14) F0 Clock Parameter
   ** 15) F1 Clock Parameter
   ** 16) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 16 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( FALSE );
   } 

   NumberOfMessages         = (WORD) sentence.Integer( 1 );
   MessageNumber            = (WORD) sentence.Integer( 2 );
   PRNNumber                = (WORD) sentence.Integer( 3 );
   WeekNumber               = (WORD) sentence.Integer( 4 );
   SVHealth                 = (WORD) ::HexValue( sentence.Field( 5 ).toLatin1() );
   Eccentricity             = (WORD) ::HexValue( sentence.Field( 6 ).toLatin1() );
   AlmanacReferenceTime     = (WORD) ::HexValue( sentence.Field( 7 ).toLatin1() );
   InclinationAngle         = (WORD) ::HexValue( sentence.Field( 8 ).toLatin1() );
   RateOfRightAscension     = (WORD) ::HexValue( sentence.Field( 9 ).toLatin1() );
   RootOfSemiMajorAxis      = ::HexValue( sentence.Field( 10 ).toLatin1() );
   ArgumentOfPerigee        = ::HexValue( sentence.Field( 11 ).toLatin1() );
   LongitudeOfAscensionNode = ::HexValue( sentence.Field( 12 ).toLatin1() );
   MeanAnomaly              = ::HexValue( sentence.Field( 13 ).toLatin1() );
   F0ClockParameter         = (WORD) ::HexValue( sentence.Field( 14 ).toLatin1() );
   F1ClockParameter         = (WORD) ::HexValue( sentence.Field( 15 ).toLatin1() );

   return( TRUE );
}

BOOL ALM::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += Hex( NumberOfMessages ); // Thanks to Chuck Shannon, cshannon@imtn.tpd.dsccc.com
   sentence += Hex( MessageNumber );    // Thanks to Chuck Shannon, cshannon@imtn.tpd.dsccc.com
   sentence += Hex( PRNNumber );        // Thanks to Chuck Shannon, cshannon@imtn.tpd.dsccc.com
   sentence += Hex( WeekNumber );       // Thanks to Chuck Shannon, cshannon@imtn.tpd.dsccc.com
   sentence += Hex( SVHealth );
   sentence += Hex( Eccentricity );
   sentence += Hex( AlmanacReferenceTime );
   sentence += Hex( InclinationAngle );
   sentence += Hex( RateOfRightAscension );
   sentence += Hex( RootOfSemiMajorAxis );
   sentence += Hex( ArgumentOfPerigee );
   sentence += Hex( LongitudeOfAscensionNode );
   sentence += Hex( MeanAnomaly );
   sentence += Hex( F0ClockParameter );
   sentence += Hex( F1ClockParameter );

   sentence.Finish();

   return( TRUE );
}

const ALM& ALM::operator = ( const ALM& source )
{
   NumberOfMessages         = source.NumberOfMessages;
   MessageNumber            = source.MessageNumber;
   PRNNumber                = source.PRNNumber;
   WeekNumber               = source.WeekNumber;
   SVHealth                 = source.SVHealth;
   Eccentricity             = source.Eccentricity;
   AlmanacReferenceTime     = source.AlmanacReferenceTime;
   InclinationAngle         = source.InclinationAngle;
   RateOfRightAscension     = source.RateOfRightAscension;
   RootOfSemiMajorAxis      = source.RootOfSemiMajorAxis;
   ArgumentOfPerigee        = source.ArgumentOfPerigee;
   LongitudeOfAscensionNode = source.LongitudeOfAscensionNode;
   MeanAnomaly              = source.MeanAnomaly;
   F0ClockParameter         = source.F0ClockParameter;
   F1ClockParameter         = source.F1ClockParameter;

   return( *this );
}
