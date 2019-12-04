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
** $Workfile: asd.cpp $
** $Revision: 5 $
** $Modtime: 10/10/98 2:45p $
*/



ASD::ASD()
{
   Mnemonic = "ASD";
}

ASD::~ASD()
{
   //Mnemonic.Empty();
}

void ASD::Empty( void )
{
}

#pragma warning( disable : 4100 )

BOOL ASD::Parse( const SENTENCE& sentence )
{
   /*
   ** ASD - Autopilot System Data
   **
   ** FORMAT TO BE DETERMINED
   **
   ** As soon as NMEA makes up their mind and publishes this we can't do anything but return TRUE
   */

   return( TRUE );
}

#pragma warning( default : 4100 )

BOOL ASD::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence.Finish();

   return( TRUE );
}
