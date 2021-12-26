#if ! defined( WPL_CLASS_HEADER )

#define WPL_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: wpl.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:43p $
*/

class WPL : public RESPONSE
{
   public:

      WPL();
      virtual ~WPL();

      /*
      ** Data
      */

      LATLONG Position;
	  QString To;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const WPL& operator = ( const WPL& source );
};

#endif // WPL_CLASS_HEADER
