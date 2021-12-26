#if ! defined( ZLZ_CLASS_HEADER )

#define ZLZ_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: zlz.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:40p $
*/

/*
** This Sentence Not Recommended For New Designs
** ZDA is recommended.
*/

class ZLZ : public RESPONSE
{
   public:

      ZLZ();
      virtual ~ZLZ();

      /*
      ** Data
      */

      QString	UTCTimeString;
      QDateTime UTCTime;
      QString	LocalTimeString;
      QDateTime LocalTime;
      int		LocalHourDeviation;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const ZLZ& operator = ( const ZLZ& source );
};

#endif // ZLZ_CLASS_HEADER
