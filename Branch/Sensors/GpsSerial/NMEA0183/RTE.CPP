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

RTE::RTE()
{
   Mnemonic = "RTE";
   Empty();
}

RTE::~RTE()
{
   Mnemonic.clear();
   Empty();
}

void RTE::Empty( void )
{
   m_TotalNumberOfMessages     = 0.0;
   m_LastMessageNumberReceived = 0.0;
   m_MessageNumber             = 0.0;
   m_LastWaypointNumberWritten = 0;

   TypeOfRoute = RouteUnknown;
   RouteName.clear();                  
   Waypoints.clear();
}

bool RTE::Parse( const SENTENCE& sentence )
{
   /*
   ** RTE - Routes
   **
   **        1   2   3 4	 5		       x    n
   **        |   |   | |    |           |    |
   ** $--RTE,x.x,x.x,a,c--c,c--c, ..... c--c*hh<CR><LF>
   **
   ** Field Number: 
   **  1) Total number of messages being transmitted
   **  2) Message Number
   **  3) Message mode
   **     c = complete route, all waypoints
   **     w = working route, the waypoint you just left, the waypoint you're heading to then all the rest
   **  4) Waypoint ID
   **  x) More Waypoints
   **  n) Checksum
   */

   Waypoints.clear();

   int field_number = 1;

   m_TotalNumberOfMessages = sentence.Double( 1 );
   
   double this_message_number = sentence.Double( 2 );
   
   if ( this_message_number == 1.0 )
   {
      /*
      ** Make sure we've got a clean list
      */

       Waypoints.clear();
   }

   std::string field_data = sentence.Field( 3 );

   if ( field_data == "c" )
   {
      TypeOfRoute = CompleteRoute;
   }
   else if ( field_data == "w" )
   {
      TypeOfRoute = WorkingRoute;
   }
   else
   {
      TypeOfRoute = RouteUnknown;
   }

   RouteName = sentence.Field( 4 );

   int number_of_data_fields = sentence.GetNumberOfDataFields();
   field_number = 5;

   std::string string_to_add;

   while( field_number < number_of_data_fields )
   {
      Waypoints.push_back(sentence.Field(field_number));
      field_number++;
   }

   return( true );
}

bool RTE::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += m_TotalNumberOfMessages;
   sentence += m_MessageNumber;

   switch( TypeOfRoute )
   {
      case CompleteRoute:

         sentence += "c";
         break;

      case WorkingRoute:

         sentence += "w";
         break;

      default:

         sentence += "";
         break;
   }

   sentence += RouteName;

   /*
   ** To Be done
   ** Take the number of entries in the list and write them out until we're done
   */

   sentence.Finish();

   return( true );
}
