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

OMEGA_PAIR::OMEGA_PAIR()
{
   Empty();
}

OMEGA_PAIR::~OMEGA_PAIR()
{
   Empty();
}

void OMEGA_PAIR::Empty( void )
{
   Name.clear();
   LaneNumber      = 0;
   CentilaneNumber = 0;
}

void OMEGA_PAIR::Parse( int first_field_number, const SENTENCE& sentence )
{
   Name            = sentence.Field( first_field_number );
   LaneNumber      = sentence.Integer( first_field_number + 1 );
   CentilaneNumber = sentence.Integer( first_field_number + 2 );
}

void OMEGA_PAIR::Write( SENTENCE& sentence )
{
   sentence += Name;
   sentence += LaneNumber;
   sentence += CentilaneNumber;
}

const OMEGA_PAIR& OMEGA_PAIR::operator = ( const OMEGA_PAIR& source )
{
   Name            = source.Name;
   LaneNumber      = source.LaneNumber;
   CentilaneNumber = source.CentilaneNumber;

   return( *this );
}
