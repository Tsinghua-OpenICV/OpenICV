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

/*
** This Sentence Not Recommended For New Designs
** Use of HDG is recommended.
*/

#include "NMEA0183.H"
#pragma hdrstop

#if defined( _DEBUG ) && defined( _INC_CRTDBG )
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#define new DEBUG_NEW
#endif // _DEBUG

HDM::HDM()
{
   Mnemonic = "HDM";
   Empty();
}

HDM::~HDM()
{
   Mnemonic.clear();
   Empty();
}

void HDM::Empty( void )
{
   HeadingDegrees = 0.0;
}

bool HDM::Parse( const SENTENCE& sentence )
{
   /*
   ** HDM - Heading - Deviation & Variation
   **
   **        1   2 3
   **        |   | |
   ** $--HDM,x.x,M*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Heading in degrees
   **  2) M, Magnetic Deviation
   **  3) Checksum
   */

   /*
   ** First we check the checksum...
   */

   if ( sentence.IsChecksumBad( 3 ) == True )
   {
      SetErrorMessage( "Invalid Checksum" );
      return( false );
   } 

   HeadingDegrees = sentence.Double( 1 );

   return( true );
}

std::string HDM::PlainEnglish( void ) const
{
   std::string return_string = "The Magnetic heading is ";

   char temp_string[ 80 ];

   ::sprintf( temp_string, "%6.2lf degrees.", HeadingDegrees );

   return_string += temp_string;

   return( return_string );
}

bool HDM::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += HeadingDegrees;
   sentence += "M";

   sentence.Finish();

   return( true );
}

const HDM& HDM::operator = ( const HDM& source )
{
   HeadingDegrees = source.HeadingDegrees;

   return( *this );
}
