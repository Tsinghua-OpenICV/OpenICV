#if ! defined( ZTE_CLASS_HEADER )

#define ZTE_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: zte.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:40p $
*/

/*
** This Sentence Not Recommended For New Designs
** ZTG is recommended.
*/

class ZTE : public RESPONSE
{
   public:

      ZTE();
      virtual ~ZTE();

      /*
      ** Data
      */

      QString	UTCTimeString;
      QDateTime UTCTime;
      QString	TimeToGoString;
      QDateTime TimeToGo;
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

      virtual const ZTE& operator = ( const ZTE& source );
};

#endif // ZTE_CLASS_HEADER
