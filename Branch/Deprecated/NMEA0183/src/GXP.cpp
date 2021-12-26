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
** $Workfile: gxp.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:44p $
*/

/*
** This Sentence Not Recommended For New Designs
** A combination of WPL, GLL, ZDA and ZTG is recommended.
*/



GXP::GXP()
{
   Mnemonic = "GXP";
}

GXP::~GXP()
{
   //Mnemonic.Empty();
}
