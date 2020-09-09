#include "nmea0183.h"
#pragma hdrstop

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1997, Samuel R. Blackburn
**
** $Workfile: manufact.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:22p $
*/


MANUFACTURER::MANUFACTURER()
{
   //Mnemonic.Empty();
   //CompanyName.Empty();
}

MANUFACTURER::MANUFACTURER( const char *mnemonic, const char *company_name )
{
   Mnemonic    = mnemonic;
   CompanyName = company_name;
}

MANUFACTURER::~MANUFACTURER()
{
   //Mnemonic.Empty();
   //CompanyName.Empty();
}
