#include "STDPOS.hpp"
//#pragma hdrstop

/*
** Author: O. Le Marchand

**
** You can use it any way you like as long as you don't try to sell it.
**

**
** $Workfile: STDPOS.cpp $
** $Revision: 6 $
** $Modtime: 14/10/08 $
*/



STDPOS::STDPOS()
{
   Mnemonic = "STDPOS";
   Empty();
}

STDPOS::~STDPOS()
{
   //Mnemonic.Empty();
   Empty();
}

void STDPOS::Empty( void )
{

	stdLat  = 0.0; 
	stdLong  = 0.0; 
	stdAlt  = 0.0; 
}

BOOL STDPOS::Parse( const SENTENCE& sentence )
{
   /*
   ** STDPOS - standard deviation on the position
   **
   **         1    2    3
   **         |    |    |
   ** $STDPOS,x.xx,y.yy,z.zz*hh<CR><LF>
   **
   ** Field Number: 
   **   1)  x.xx  is the latitude std dev
		2)  y.yy  is the longitude std dev
		3)  is the altitude std dev
		
		
		
   */

    stdLat   = sentence.Double( 1 );
	stdLong  = sentence.Double( 2 );
	stdAlt   = sentence.Double( 3 );

   return( TRUE );
}

QString STDPOS::PlainEnglish( void ) const
{
   QString return_string;

   return_string = "not yet available ";

   return( return_string );
}

BOOL STDPOS::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += stdLat;
   sentence += stdLong;
   sentence += stdAlt;

   sentence.Finish();

   return( TRUE );
}

const STDPOS& STDPOS::operator = ( const STDPOS& source )
{
    stdLat    =  source.stdLat;
    stdLong   =  source.stdLong;
	stdAlt    =  source.stdAlt;

   return( *this );
}
