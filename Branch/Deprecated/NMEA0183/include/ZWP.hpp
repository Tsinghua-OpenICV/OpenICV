#if ! defined( ZWP_CLASS_HEADER )

#define ZWP_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: zwp.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:37p $
*/

/*
** This Sentence Not Recommended For New Designs
** ZTG is recommended.
*/

class ZWP : public RESPONSE
{
   public:

      ZWP();
      virtual ~ZWP();

      /*
      ** Data
      */

      QString	UTCTimeString;
      QDateTime UTCTime;
      QString	ArrivalTimeString;
      QDateTime ArrivalTime;
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

      virtual const ZWP& operator = ( const ZWP& source );
};

#endif // ZWP_CLASS_HEADER
