#include "HEAVE.hpp"
//#pragma hdrstop

/*
** Author: O. Le Marchand

**
** You can use it any way you like as long as you don't try to sell it.
**

**
** $Workfile: HEAVE.cpp $
** $Revision: 6 $
** $Modtime: 14/10/08 $
*/



HEAVE::HEAVE()
{
   Mnemonic = "HEAVE";
   Empty();
}

HEAVE::~HEAVE()
{
   //Mnemonic.Empty();
   Empty();
}

void HEAVE::Empty( void )
{

	surge = 0.0; 
	sway  = 0.0; 
	heave = 0.0; 
}

BOOL HEAVE::Parse( const SENTENCE& sentence )
{
   /*
   ** HEAVE - Heave
   **
   **         1     2     3      
   **         |     |     |     
   ** $HEAVE_,x.xxx,y.yyy,z.zzz<CR><LF>
   **
   ** Field Number: 
   **   1)  x.xxx  is the surge
		2)  y.yyy  is the sway
		3)  z.zzz  is the heave
		
   */

    surge = sentence.Double( 1 );
    sway  = sentence.Double( 2 );
    heave = sentence.Double( 3 );
   return( TRUE );
}

QString HEAVE::PlainEnglish( void ) const
{
   QString return_string;

   return_string = "not yet available ";

   return( return_string );
}

BOOL HEAVE::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += surge;
   sentence += sway;
   sentence += heave;

   sentence.Finish();

   return( TRUE );
}

const HEAVE& HEAVE::operator = ( const HEAVE& source )
{
    surge  =  source.surge;
    sway   =  source.sway;
	heave   =  source.heave;

   return( *this );
}
