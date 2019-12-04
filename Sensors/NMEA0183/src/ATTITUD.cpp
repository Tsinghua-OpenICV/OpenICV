#include "ATTITUD.hpp"
//#pragma hdrstop

/*
** Author: O. Le Marchand

**
** You can use it any way you like as long as you don't try to sell it.
**

**
** $Workfile: ATTITUD.cpp $
** $Revision: 6 $
** $Modtime: 14/10/08 $
*/



ATTITUD::ATTITUD()
{
   Mnemonic = "ATTITUD";
   Empty();
}

ATTITUD::~ATTITUD()
{
   //Mnemonic.Empty();
   Empty();
}

void ATTITUD::Empty( void )
{
	roll = 0.0; 
	pitch = 0.0; 
}

BOOL ATTITUD::Parse( const SENTENCE& sentence )
{
   /*
   ** ATTITUD - Attitude
   **
   **          1     2     
   **          |     |     
   ** $ATTITUD,x.xxx,y.yyy<CR><LF>
   **
   ** Field Number: 
   **   1)  x.xxx  is the roll in degrees
		2)  y.yyy  is the pitch in degrees
   */

    roll    = sentence.Double( 1 );
	pitch   = sentence.Double( 2 );

   return( TRUE );
}

QString ATTITUD::PlainEnglish( void ) const
{
   QString return_string;

   return_string = "not yet available ";

   return( return_string );
}

BOOL ATTITUD::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += roll;
   sentence += pitch;

   sentence.Finish();

   return( TRUE );
}

const ATTITUD& ATTITUD::operator = ( const ATTITUD& source )
{
    roll   =  source.roll;
    pitch  =  source.pitch;

   return( *this );
}
