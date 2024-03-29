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

HDG::HDG()
{
   Mnemonic = "HDG";
   Empty();
}

HDG::~HDG()
{
   Mnemonic.clear();
   Empty();
}

void HDG::Empty( void )
{
   MagneticSensorHeadingDegrees = 0.0;
   MagneticDeviationDegrees     = 0.0;
   MagneticDeviationDirection   = EW_Unknown;
   MagneticVariationDegrees     = 0.0;
   MagneticVariationDirection   = EW_Unknown;
}

bool HDG::Parse( const SENTENCE& sentence )
{
   /*
   ** HDG - Heading - Deviation & Variation
   **
   **        1   2   3 4   5 6
   **        |   |   | |   | |
   ** $--HDG,x.x,x.x,a,x.x,a*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Magnetic Sensor heading in degrees
   **  2) Magnetic Deviation, degrees
   **  3) Magnetic Deviation direction, E = Easterly, W = Westerly
   **  4) Magnetic Variation degrees
   **  5) Magnetic Variation direction, E = Easterly, W = Westerly
   **  6) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 6 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( false );
   } 

   MagneticSensorHeadingDegrees = sentence.Double( 1 );
   MagneticDeviationDegrees     = sentence.Double( 2 );
   MagneticDeviationDirection   = sentence.EastOrWest( 3 );
   MagneticVariationDegrees     = sentence.Double( 4 );
   MagneticVariationDirection   = sentence.EastOrWest( 5 );

   return( true );
}

bool HDG::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += MagneticSensorHeadingDegrees;
   sentence += MagneticDeviationDegrees;
   sentence += MagneticDeviationDirection;
   sentence += MagneticVariationDegrees;
   sentence += MagneticVariationDirection;

   sentence.Finish();

   return( true );
}

const HDG& HDG::operator = ( const HDG& source )
{
   MagneticSensorHeadingDegrees = source.MagneticSensorHeadingDegrees;
   MagneticDeviationDegrees     = source.MagneticDeviationDegrees;
   MagneticDeviationDirection   = source.MagneticDeviationDirection;
   MagneticVariationDegrees     = source.MagneticVariationDegrees;
   MagneticVariationDirection   = source.MagneticVariationDirection;

   return( *this );
}
