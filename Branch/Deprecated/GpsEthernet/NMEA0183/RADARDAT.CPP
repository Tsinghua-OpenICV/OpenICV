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

RADAR_DATA::RADAR_DATA()
{
   Empty();
}

RADAR_DATA::~RADAR_DATA()
{
   Empty();
}

void RADAR_DATA::Empty( void )
{
   OriginRangeFromOwnShip       = 0.0;
   OriginBearingDegreesFromZero = 0.0;
   VariableRangeMarkerRange     = 0.0;
   BearingLineDegreesFromZero   = 0.0;
}

void RADAR_DATA::Parse( int first_field_number, const SENTENCE& sentence )
{
   OriginRangeFromOwnShip       = sentence.Double( first_field_number );
   OriginBearingDegreesFromZero = sentence.Double( first_field_number + 1 );
   VariableRangeMarkerRange     = sentence.Double( first_field_number + 2 );
   BearingLineDegreesFromZero   = sentence.Double( first_field_number + 3 );
}

void RADAR_DATA::Write( SENTENCE& sentence )
{
   sentence += OriginRangeFromOwnShip;
   sentence += OriginBearingDegreesFromZero;
   sentence += VariableRangeMarkerRange;
   sentence += BearingLineDegreesFromZero;
}

const RADAR_DATA& RADAR_DATA::operator = ( const RADAR_DATA& source )
{
   OriginRangeFromOwnShip       = source.OriginRangeFromOwnShip;
   OriginBearingDegreesFromZero = source.OriginBearingDegreesFromZero;
   VariableRangeMarkerRange     = source.VariableRangeMarkerRange;
   BearingLineDegreesFromZero   = source.BearingLineDegreesFromZero;

   return( *this );
}
