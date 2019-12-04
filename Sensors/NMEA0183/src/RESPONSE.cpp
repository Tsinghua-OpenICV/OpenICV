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
** $Workfile: response.cpp $
** $Revision: 6 $
** $Modtime: 10/10/98 2:51p $
*/


RESPONSE::RESPONSE()
{
   DataSource = "--"; // Default to an unknown source
   //Mnemonic.Empty();
   //Talker.Empty();

   // 15 Jan 98 - Thanks go to Craig Miller (Craig.Miller@Bigfoot.com) for
   // finding a bug here. I had left off the () after Empty
   //ErrorMessage.Empty();
}

RESPONSE::~RESPONSE()
{
   //DataSource.Empty();
   //Mnemonic.Empty();
   //Talker.Empty();
   //ErrorMessage.Empty();
}

void RESPONSE::SetContainer( NMEA0183 *container )
{
   container_p = container;
}

void RESPONSE::SetErrorMessage( const QString& error_message )
{
   ErrorMessage  = Mnemonic;
   ErrorMessage += ", ";
   ErrorMessage += error_message;
}

BOOL RESPONSE::Write( SENTENCE& sentence )
{
   /*
   ** All NMEA0183 sentences begin with the mnemonic...
   **
   ** Thanks to Jan-Erik Eriksson (Jan-Erik.Eriksson@st.se) for
   ** finding and fixing a bug here
   */

   sentence  = "$" + DataSource + Mnemonic;

   return( TRUE );
}

QString RESPONSE::PlainEnglish( void ) const
{
   QString return_string;

   //return_string.Empty();

   return( return_string );
}
