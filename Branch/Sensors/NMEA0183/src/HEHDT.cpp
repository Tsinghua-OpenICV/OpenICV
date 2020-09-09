#include "HEHDT.hpp"
//#pragma hdrstop

/*
** Author: O. Le Marchand

**
** You can use it any way you like as long as you don't try to sell it.
**

**
** $Workfile: HEHDT.cpp $
** $Revision: 6 $
** $Modtime: 14/10/08 $
*/



HEHDT::HEHDT()
{
   Mnemonic = "HEHDT";
   Empty();
}

HEHDT::~HEHDT()
{
   //Mnemonic.Empty();
   Empty();
}

void HEHDT::Empty( void )
{
	
   heading  = 0.0; 
}

BOOL HEHDT::Parse( const SENTENCE& sentence )
{
   /*
   ** HEHDT - Heading
   **
   **         1
   **         |  
   ** $HEHDT,x.xxx,T*hh
   **
   ** Field Number: 
   **   1)  x.xxx	is the true heading in degrees
		
		
   */

    heading = sentence.Double( 1 );

   return( TRUE );
}

QString HEHDT::PlainEnglish( void ) const
{
   QString return_string;

   return_string = "not yet available ";

   return( return_string );
}

BOOL HEHDT::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += heading;

   sentence.Finish();

   return( TRUE );
}

const HEHDT& HEHDT::operator = ( const HEHDT& source )
{
    heading =  source.heading;

   return( *this );
}
