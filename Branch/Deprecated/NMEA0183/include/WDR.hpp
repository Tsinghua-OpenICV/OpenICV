#if ! defined( WDR_CLASS_HEADER )

#define WDR_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: wdr.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:43p $
*/

/*
** This Sentence Not Recommended For New Designs
** WDR is recommended (yes, WDR is another sentence not recommended for
** new designs but that's what the spec says).
*/

class WDR : public RESPONSE
{
   public:

      WDR();
      virtual ~WDR();

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

      virtual const WDR& operator = ( const WDR& source );
};

#endif // WDR_CLASS_HEADER
