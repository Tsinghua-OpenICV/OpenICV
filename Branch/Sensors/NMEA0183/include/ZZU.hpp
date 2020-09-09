#if ! defined( ZZU_CLASS_HEADER )

#define ZZU_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: zzu.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:37p $
*/

/*
** This Sentence Not Recommended For New Designs
** ZDA is recommended.
*/

class ZZU : public RESPONSE
{
   public:

      ZZU();
      virtual ~ZZU();

      /*
      ** Data
      */

      QString	UTCTimeString;
      QDateTime UTCTime;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const ZZU& operator = ( const ZZU& source );
};

#endif // ZZU_CLASS_HEADER
