#if ! defined( GTD_CLASS_HEADER )

#define GTD_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: gtd.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:50p $
*/

class GTD : public RESPONSE
{
   public:

      GTD();
      virtual ~GTD();

      /*
      ** Data
      */
       
      QString TimeDifference1;
      QString TimeDifference2;
      QString TimeDifference3;
      QString TimeDifference4;
      QString TimeDifference5;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const GTD& operator = ( const GTD& source );
};

#endif // GTD_CLASS_HEADER
