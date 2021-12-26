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
** $Workfile: rte.cpp $
** $Revision: 5 $
** $Modtime: 10/10/98 4:21p $
*/


RTE::RTE()
{
   Mnemonic = "RTE";
   Empty();
}

RTE::~RTE()
{
   //Mnemonic.Empty();
   Empty();
}

void RTE::Empty( void )
{
   m_TotalNumberOfMessages     = 0.0;
   m_LastMessageNumberReceived = 0.0;
   m_MessageNumber             = 0.0;
   m_LastWaypointNumberWritten = 0;

   TypeOfRoute = RouteUnknown;
   //RouteName.Empty();
                  
   m_DeleteAllEntries();
}

BOOL RTE::Parse( const SENTENCE& sentence )
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

   m_DeleteAllEntries();

   int field_number = 1;

   m_TotalNumberOfMessages = sentence.Double( 1 );
   
   double this_message_number = sentence.Double( 2 );
   
   if ( this_message_number == 1.0 )
   {
      /*
      ** Make sure we've got a clean list
      */

      m_DeleteAllEntries();
   }

   QString field_data = sentence.Field( 3 );

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

   QString * string_to_add_p = (QString *) NULL;

   while( field_number < number_of_data_fields )
   {
      string_to_add_p = new QString;

      *string_to_add_p = sentence.Field( field_number );
	  
	  Waypoints.resize(Waypoints.size()+1);
      Waypoints.insert( Waypoints.size(),string_to_add_p  );
      //Waypoints.Add( string_to_add_p );

      string_to_add_p = (QString *) NULL;

      field_number++;
   }

   return( TRUE );
}

BOOL RTE::Write( SENTENCE& sentence )
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

   return( TRUE );
}

void RTE::m_DeleteAllEntries( void )
{
   int loop_index = 0;
   int number_of_entries = Waypoints.size();

   QString * entry_p = (QString *) NULL;

   /*while( loop_index < number_of_entries )
   {
      entry_p = (QString *) Waypoints[ loop_index ];
      Waypoints[ loop_index ] = NULL;
      delete entry_p;
      entry_p = (QString *) NULL;
      loop_index++;
   }*/

   Waypoints.clear();
}
