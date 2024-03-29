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

LONGITUDE::LONGITUDE()
{
   Empty();
}

LONGITUDE::~LONGITUDE()
{
   Empty();
}

void LONGITUDE::Empty( void )
{
   Longitude = 0.0;
   Easting   = EW_Unknown;
}

bool LONGITUDE::IsDataValid( void )
{
   if ( Easting != East && Easting != West )
   {
      return( false );
   }

   return( true );
}

void LONGITUDE::Parse( int position_field_number, int east_or_west_field_number, const SENTENCE& sentence )
{
   // Thanks go to Eric Parsonage (ericpa@mpx.com.au) for finding a nasty
   // little bug that used to live here.

   double position = 0.0;

   position = sentence.Double( position_field_number );

   std::string east_or_west = sentence.Field( east_or_west_field_number );

   Set( position, east_or_west.c_str() );
}

void LONGITUDE::Set( double position, const char *east_or_west )
{
   Longitude  = position;
   Coordinate = position;

   if ( east_or_west[ 0 ] == 'E' )
   {
      Easting = East;
   }
   else if ( east_or_west[ 0 ] == 'W' )
   {
      Easting = West;
   }
   else
   {
      Easting = EW_Unknown;
   }
}

void LONGITUDE::Write( SENTENCE& sentence )
{
   char temp_string[ 80 ];

   ::sprintf( temp_string, "%08.2f", Longitude );
   sentence += temp_string;
   
   if ( Easting == East )
   {
      sentence += "E";
   }
   else if ( Easting == West )
   {
      sentence += "W";
   }
   else
   {
      /*
      ** Thanks to Jan-Erik Eriksson (Jan-Erik.Eriksson@st.se) for
      ** finding and fixing a bug here
      */

      sentence += "";
   }
}

const LONGITUDE& LONGITUDE::operator = ( const LONGITUDE& source )
{
   Longitude = source.Longitude;
   Easting   = source.Easting;

   return( *this );
}
