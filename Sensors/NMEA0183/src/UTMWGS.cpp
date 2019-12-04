#include "UTMWGS.hpp"
//#pragma hdrstop

/*
** Author: O. Le Marchand

**
** You can use it any way you like as long as you don't try to sell it.
**

**
** $Workfile: UTMWGS.cpp $
** $Revision: 6 $
** $Modtime: 14/10/08 $
*/



UTMWGS::UTMWGS()
{
   Mnemonic = "UTMWGS";
   Empty();
}

UTMWGS::~UTMWGS()
{
   //Mnemonic.Empty();
   Empty();
}

void UTMWGS::Empty( void )
{

	
   //UTCTime.Empty();
	UTMzoneChar					= ' ';
	UTMzone						= 0; 
    east						= 0.0;
	north						= 0.0;
    up							= 0.0;

}

BOOL UTMWGS::Parse( const SENTENCE& sentence )
{
   /*
   ** UTMWGS
   **         1 2  3     4     5
   **         | |  |     |     |
   ** $UTMWGS,c,nn,x.xxx,y.yyy,z.zzz*hh<CR><LF>
   **
   ** Field Number: 
   **   1)  c and nn	represent the map, Zone/area
		3)  x.xxx		is the east position in meter
		4)  y.yyy		is the north position in meter	
		5)  z.zzz		is the altitude in meters
		
		
   */

	
	UTMzoneChar					= sentence.Field(1).at(0);
	UTMzone						= sentence.Integer( 2 ); 
    east						= sentence.Double( 3 );
	north						= sentence.Double( 4 );
    up							= sentence.Double( 5 );

   return( TRUE );
}

QString UTMWGS::PlainEnglish( void ) const
{
   QString return_string;

   return_string = "not yet available ";

   return( return_string );
}

BOOL UTMWGS::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

	sentence += UTMzoneChar;
	sentence += UTMzone; 
    sentence += east;
	sentence += north;
    sentence += up;

   sentence.Finish();

   return( TRUE );
}

const UTMWGS& UTMWGS::operator = ( const UTMWGS& source )
{
    UTMzoneChar					= source.UTMzoneChar;
	UTMzone						= source.UTMzone; 
    east						= source.east;
	north						= source.north;
    up							= source.up;

   return( *this );
}
