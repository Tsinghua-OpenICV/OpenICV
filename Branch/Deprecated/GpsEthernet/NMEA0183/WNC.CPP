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

WNC::WNC()
{
   Mnemonic = "WNC";
   Empty();
}

WNC::~WNC()
{
   Mnemonic.clear();
   Empty();
}

void WNC::Empty( void )
{
   MilesDistance      = 0.0;
   KilometersDistance = 0.0;
   To.clear();
   From.clear();
}

bool WNC::Parse( const SENTENCE& sentence )
{
   /*
   ** WNC - Distance - Waypoint to Waypoint
   **
   **        1   2 3   4 5    6    7
   **        |   | |   | |    |    |
   ** $--WNC,x.x,N,x.x,K,c--c,c--c*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Distance, Nautical Miles
   **  2) N = Nautical Miles
   **  3) Distance, Kilometers
   **  4) K = Kilometers
   **  5) TO Waypoint
   **  6) FROM Waypoint
   **  7) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 7 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( false );
   } 

   MilesDistance      = sentence.Double( 1 );
   KilometersDistance = sentence.Double( 3 );
   To                 = sentence.Field( 5 );
   From               = sentence.Field( 6 );

   return( true );
}

bool WNC::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += MilesDistance;
   sentence += "N";
   sentence += KilometersDistance;
   sentence += "K";
   sentence += To;
   sentence += From;

   sentence.Finish();

   return( true );
}

const WNC& WNC::operator = ( const WNC& source )
{
   MilesDistance      = source.MilesDistance;
   KilometersDistance = source.KilometersDistance;
   To                 = source.To;
   From               = source.From;

   return( *this );
}
