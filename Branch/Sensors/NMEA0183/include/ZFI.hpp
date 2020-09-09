#if ! defined( ZFI_CLASS_HEADER )

#define ZFI_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: zfi.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:48p $
*/

/*
** This Sentence Not Recommended For New Designs
** ZFO is recommended.
*/

class ZFI : public RESPONSE
{
   public:

      ZFI();
      virtual ~ZFI();

      /*
      ** Data
      */

      QString	UTCTimeString;
      QDateTime UTCTime;
      QString	ElapsedTimeString;
      QDateTime ElapsedTime;
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

      virtual const ZFI& operator = ( const ZFI& source );
};

#endif // ZFI_CLASS_HEADER
