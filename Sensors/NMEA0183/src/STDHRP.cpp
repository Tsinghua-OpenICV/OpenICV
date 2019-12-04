#include "STDHRP.hpp"
//#pragma hdrstop

/*
** Author: O. Le Marchand

**
** You can use it any way you like as long as you don't try to sell it.
**

**
** $Workfile: STDHRP.cpp $
** $Revision: 6 $
** $Modtime: 14/10/08 $
*/



STDHRP::STDHRP()
{
   Mnemonic = "STDHRP";
   Empty();
}

STDHRP::~STDHRP()
{
   //Mnemonic.Empty();
   Empty();
}

void STDHRP::Empty( void )
{

	stdHeading  = 0.0; 
	stdRoll  = 0.0; 
	stdPitch  = 0.0; 
}

BOOL STDHRP::Parse( const SENTENCE& sentence )
{
   /*
   ** STDHRP - Global Positioning System Fix Data
   ** Time, Position and fix related data fora GPS receiver.
   **
   **         1     2     3 
   **         |     |     | 
   ** $STDHRP,x.xxx,y.yyy,z.zzz*hh<CR><LF>
   **
   ** Field Number: 
   **   1)  x.xxx  is the heading std dev
		2)  y.yyy  is the roll std dev
		3)  z.zzz  is the pitch std dev
		
		
		
   */

    stdHeading = sentence.Double( 1 );
	stdRoll    = sentence.Double( 1 );
	stdPitch   = sentence.Double( 1 );

   return( TRUE );
}

QString STDHRP::PlainEnglish( void ) const
{
   QString return_string;

   return_string = "not yet available ";

   return( return_string );
}

BOOL STDHRP::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += stdHeading;
   sentence += stdRoll;
   sentence += stdPitch;

   sentence.Finish();

   return( TRUE );
}

const STDHRP& STDHRP::operator = ( const STDHRP& source )
{
    stdHeading =  source.stdHeading;
    stdRoll    =  source.stdRoll;
	stdPitch   =  source.stdPitch;

   return( *this );
}
