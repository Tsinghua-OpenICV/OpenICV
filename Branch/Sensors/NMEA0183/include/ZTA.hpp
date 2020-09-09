#if ! defined( ZTA_CLASS_HEADER )

#define ZTA_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: zta.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:38p $
*/

/*
** This Sentence Not Recommended For New Designs
** ZTG is recommended.
*/

class ZTA : public RESPONSE
{
   public:

      ZTA();
      virtual ~ZTA();

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

      virtual const ZTA& operator = ( const ZTA& source );
};

#endif // ZTA_CLASS_HEADER
