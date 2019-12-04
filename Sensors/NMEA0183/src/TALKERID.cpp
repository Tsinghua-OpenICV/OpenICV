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
** $Workfile: talkerid.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:57p $
*/

QString talker_id( const QString &sentence )
{
   QString return_string;

   //return_string.Empty();

   if ( sentence.length() >= 3 )
   {
      if ( sentence[ 0 ] == '$' )
      {
         return_string = sentence.mid( 1, 2 );
      }
   }

   return( return_string );
}
