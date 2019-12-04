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
** $Workfile: hex.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:58p $
*/

QString Hex( unsigned int value )
{
   QString return_string;
   return_string =  QString::number(value);
   //return_string.Format( "%04lX",value  );

   return( return_string );
}
