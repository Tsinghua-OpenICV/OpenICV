#include "SPEED.hpp"
//#pragma hdrstop

/*
** Author: O. Le Marchand

**
** You can use it any way you like as long as you don't try to sell it.
**

**
** $Workfile: SPEED.cpp $
** $Revision: 6 $
** $Modtime: 14/10/08 $
*/



SPEED::SPEED()
{
   Mnemonic = "SPEED_";
   Empty();
}

SPEED::~SPEED()
{
   //Mnemonic.Empty();
   Empty();
}

void SPEED::Empty( void )
{
	east   = 0.0; 
	north  = 0.0; 
	up     = 0.0; 
}

BOOL SPEED::Parse( const SENTENCE& sentence )
{
   /*
   ** SPEED - Speed
   **
   **         1     2     3    
   **         |     |     |    
   ** $SPEED_,x.xxx,y.yyy,z.zzz*hh<CR><LF>
   **
   ** Field Number: 
		1)  x.xxx   is the east speed in m/s
		2)  y.yyy	is the north speed in m/s
		3)  z.zzz   is the vertical (up) speed in m/s	
		
   */

    east   = sentence.Double( 1 );
	north  = sentence.Double( 2 );
	up     = sentence.Double( 3 );

   return( TRUE );
}

QString SPEED::PlainEnglish( void ) const
{
   QString return_string;

   return_string = "not yet available ";

   return( return_string );
}

BOOL SPEED::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += east;
   sentence += north;
   sentence += up;

   sentence.Finish();

   return( TRUE );
}

const SPEED& SPEED::operator = ( const SPEED& source )
{
    east  =  source.east;
	north =  source.north;
	up    =  source.up;
    

   return( *this );
}
