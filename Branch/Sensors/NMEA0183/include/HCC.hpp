#if ! defined( HCC_CLASS_HEADER )

#define HCC_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: hcc.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:48p $
*/

/*
** This Sentence Not Recommended For New Designs
** Use of HDG is recommended.
*/

class HCC : public RESPONSE
{
   public:

      HCC();
      virtual ~HCC();

      /*
      ** Data
      */

      double HeadingDegrees;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual QString PlainEnglish( void ) const;
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const HCC& operator = ( const HCC& source );
};

#endif // HCC_CLASS_HEADER
