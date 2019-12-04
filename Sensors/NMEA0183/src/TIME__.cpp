#include "TIME__.hpp"
//#pragma hdrstop

/*
** Author: O. Le Marchand

**
** You can use it any way you like as long as you don't try to sell it.
**

**
** $Workfile: TIME_.cpp $
** $Revision: 6 $
** $Modtime: 14/10/08 $
*/



TIME_::TIME_()
{
   Mnemonic = "TIME__";
   Empty();
}

TIME_::~TIME_()
{
   //Mnemonic.Empty();
   Empty();
}

void TIME_::Empty( void )
{

	
   //UTCTime.Empty();
}

BOOL TIME_::Parse( const SENTENCE& sentence )
{
   /*
   ** TIME_ 
   **
   **          1
   **          |
   ** $TIME__, hhmmss.sss*hh<CR><LF>
   **
   ** Field Number: 
   **   1)  hhmmss.ss	is the UTC absolute time
		
   */

    UTCTime                    = sentence.Field( 1 );
	Time					   = sentence.Time( 1 );

   return( TRUE );
}

QString TIME_::PlainEnglish( void ) const
{
   QString return_string;

   return_string = "not yet available ";

   return( return_string );
}

BOOL TIME_::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += UTCTime;

   sentence.Finish();

   return( TRUE );
}

const TIME_& TIME_::operator = ( const TIME_& source )
{
    UTCTime                    =  source.UTCTime;

   return( *this );
}
