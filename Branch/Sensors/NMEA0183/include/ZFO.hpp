#if ! defined( ZFO_CLASS_HEADER )

#define ZFO_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: zfo.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:48p $
*/

class ZFO : public RESPONSE
{
   public:

      ZFO();
      virtual ~ZFO();

      /*
      ** Data
      */

      QString	UTCTime;
      QDateTime Time;
      QString	ElapsedTime;
      QString	From;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const ZFO& operator = ( const ZFO& source );
};

#endif // ZFO_CLASS_HEADER
