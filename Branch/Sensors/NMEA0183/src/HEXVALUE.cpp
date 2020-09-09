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
** $Workfile: hexvalue.cpp $
** $Revision: 3 $
** $Modtime: 10/09/98 7:10p $
*/

#ifdef _MSC_VER
#   pragma warning(disable:4996)
#endif // _MSC_VER

unsigned int HexValue( const char *hex_string )
{
   unsigned int return_value = 0;

   sscanf( hex_string, "%x", (unsigned int *) &return_value );

   return( return_value );
}
