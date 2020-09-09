#if ! defined( TEP_CLASS_HEADER )

#define TEP_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: tep.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:44p $
*/

/*
** This Sentence Not Recommended For New Designs
** There is no recommended replacement.
*/

class TEP : public RESPONSE
{
   public:

      TEP();
      virtual ~TEP();

      /*
      ** Data
      */

      double ElevationDegrees;

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

      virtual const TEP& operator = ( const TEP& source );
};

#endif // TEP_CLASS_HEADER
