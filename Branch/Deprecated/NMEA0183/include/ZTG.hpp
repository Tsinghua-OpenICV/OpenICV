#if ! defined( ZTG_CLASS_HEADER )

#define ZTG_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: ztg.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:38p $
*/

class ZTG : public RESPONSE
{
   public:

      ZTG();
      virtual ~ZTG();

      /*
      ** Data
      */

      QString	UTCTime;
      QDateTime Time;
      QString	TimeRemaining;
      QString	To;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const ZTG& operator = ( const ZTG& source );
};

#endif // ZTG_CLASS_HEADER
