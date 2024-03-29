/*
Author: Samuel R. Blackburn
Internet: wfc@pobox.com

"You can get credit for something or get it done, but not both."
Dr. Richard Garwin

The MIT License (MIT)

Copyright (c) 1996-2015 Sam Blackburn

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "NMEA0183.H"
#pragma hdrstop

#if defined( _DEBUG ) && defined( _INC_CRTDBG )
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#define new DEBUG_NEW
#endif // _DEBUG

AAM::AAM()
{
   Mnemonic = "AAM";
   Empty();
}

AAM::~AAM()
{
   Mnemonic.clear();
   Empty();
}

void AAM::Empty( void )
{
   IsArrivalCircleEntered = NMEA_Unknown;
   IsPerpendicular        = NMEA_Unknown;
   CircleRadius           = 0.0;
   WaypointID.clear();
}

bool AAM::Parse( const SENTENCE& sentence )
{
   /*
   ** AAM - Waypoint Arrival Alarm
   **
   **        1 2 3   4 5    6
   **        | | |   | |    |
   ** $--AAM,A,A,x.x,N,c--c*hh<CR><LF>
   **
   ** 1) Status, A = Arrival circle entered
   ** 2) Status, A = perpendicular passed at waypoint
   ** 3) Arrival circle radius
   ** 4) Units of radius, nautical miles
   ** 5) Waypoint ID
   ** 6) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 6 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( false );
   } 

   /*
   ** Line has already been checked for checksum validity
   */

   IsArrivalCircleEntered = sentence.Boolean( 1 );
   IsPerpendicular        = sentence.Boolean( 2 );
   CircleRadius           = sentence.Double( 3 );
   WaypointID             = sentence.Field( 5 );

   return( true );
}

bool AAM::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += IsArrivalCircleEntered;
   sentence += IsPerpendicular;
   sentence += CircleRadius;
   sentence += "N";
   sentence += WaypointID;

   sentence.Finish();

   return( true );
}

const AAM& AAM::operator = ( const AAM& source )
{
   IsArrivalCircleEntered = source.IsArrivalCircleEntered;
   IsPerpendicular        = source.IsPerpendicular;
   CircleRadius           = source.CircleRadius;
   WaypointID             = source.WaypointID;

   return( *this );
}
