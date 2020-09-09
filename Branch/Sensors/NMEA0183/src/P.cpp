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
** $Workfile: p.cpp $
** $Revision: 4 $
** $Modtime: 10/10/98 2:42p $
*/


P::P()
{
   Mnemonic = "P";
   Empty();
}

P::~P()
{
   //Mnemonic.Empty();
   Empty();
}

void P::Empty( void )
{
   //Sentence.Empty();
   //Source.Empty();
}

BOOL P::Parse( const SENTENCE& sentence )
{
   /*
   ** This is where parsing of proprietary sentences will go...
   */

   Sentence = sentence;

   QString temp_string = sentence.Field( 0 );

   Source      = temp_string.mid( 1, 3 );
   CompanyName = Manufacturers[ Source.toLatin1() ];

   //TRACE1( "Source is   \"%s\"\n", (const char *) Source );
   //TRACE1( "CompanyName \"%s\"\n", (const char *) CompanyName );

   return( TRUE );
}

BOOL P::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += Sentence;

   sentence.Finish();

   return( TRUE );
}

const P& P::operator = ( const P& source )
{
   Sentence = source.Sentence;

   return( *this );
}
