#if ! defined( WDC_CLASS_HEADER )

#define WDC_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: wdc.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:43p $
*/

/*
** This Sentence Not Recommended For New Designs
** BWC is recommended.
*/

class WDC : public RESPONSE
{
   public:

      WDC();
      virtual ~WDC();

      /*
      ** Data
      */

      double  NauticalMiles;
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

      virtual const WDC& operator = ( const WDC& source );
};

#endif // WDC_CLASS_HEADER
