#include "POSITI.hpp"
//#pragma hdrstop

/*
** Author: O. Le Marchand

**
** You can use it any way you like as long as you don't try to sell it.
**

**
** $Workfile: POSITI.cpp $
** $Revision: 6 $
** $Modtime: 14/10/08 $
*/



POSITI::POSITI()
{
   Mnemonic = "POSITI";
   Empty();
}

POSITI::~POSITI()
{
   //Mnemonic.Empty();
   Empty();
}

void POSITI::Empty( void )
{

	latitude  = 0.0; 
	longitude = 0.0; 
	altitude  = 0.0; 
}

BOOL POSITI::Parse( const SENTENCE& sentence )
{
   /*
   ** POSITI - Global Positioning System Fix Data
   ** Time, Position and fix related data fora GPS receiver.
   **
   **         1         2         3        
   **         |         |         |        
   ** $POSITI,x.xxxxxxx,y.yyyyyyy,z.zzz*hh<CR><LF>
   **
   ** Field Number: 
		1)  x.xxxxxxx  is the latitude in deg
		2)  y.yyyyyyy  is the longitude in degrees
		3)  z.zzz      is the altitude in meters
		
		
   */

    latitude   = sentence.Double( 1 );
	longitude  = sentence.Double( 2 );
	altitude   = sentence.Double( 3 );

   return( TRUE );
}

QString POSITI::PlainEnglish( void ) const
{
   QString return_string;

   return_string = "not yet available ";

   return( return_string );
}

BOOL POSITI::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += latitude;
   sentence += longitude;
   sentence += altitude;

   sentence.Finish();

   return( TRUE );
}

const POSITI& POSITI::operator = ( const POSITI& source )
{
    latitude    =  source.latitude;
    longitude	=  source.longitude;
	altitude	=  source.altitude;

   return( *this );
}
