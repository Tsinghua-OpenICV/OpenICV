#include "STDSPD.hpp"
//#pragma hdrstop

/*
** Author: O. Le Marchand

**
** You can use it any way you like as long as you don't try to sell it.
**

**
** $Workfile: STDSPD.cpp $
** $Revision: 6 $
** $Modtime: 14/10/08 $
*/



STDSPD::STDSPD()
{
   Mnemonic = "STDSPD";
   Empty();
}

STDSPD::~STDSPD()
{
   //Mnemonic.Empty();
   Empty();
}

void STDSPD::Empty( void )
{

    stdHdg                    = 0.0;
	stdRoll                    = 0.0;
	stdPitch                    = 0.0;
}

BOOL STDSPD::Parse( const SENTENCE& sentence )
{
   /*
   ** STDSPD 
   **         1    2    3
   **         |    |    |
   ** $STDPOS,x.xx,y.yy,z.zz*hh<CR><LF>
   **
   ** Field Number: 
   **   1)  x.xx	is the heading standard deviation
		2)  y.yy	is the roll standard deviation
		3)  z.zz	is the ptich standard deviation		
   */

    stdHdg                    = sentence.Double( 1 );
	stdRoll                   = sentence.Double( 2 );
	stdPitch                  = sentence.Double( 3 );

   return( TRUE );
}

QString STDSPD::PlainEnglish( void ) const
{
   QString return_string;

   return_string = "not yet available ";

   return( return_string );
}

BOOL STDSPD::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   
     sentence += stdHdg;
	 sentence += stdRoll;
	 sentence += stdPitch;

	 sentence.Finish();

   return( TRUE );
}

const STDSPD& STDSPD::operator = ( const STDSPD& source )
{
    
    stdHdg                    = source.stdHdg;
	stdRoll                    = source.stdRoll;
	stdPitch                    = source.stdPitch;

   return( *this );
}
