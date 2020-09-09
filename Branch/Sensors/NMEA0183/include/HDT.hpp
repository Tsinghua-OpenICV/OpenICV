#if ! defined( HDT_CLASS_HEADER )

#define HDT_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: hdt.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:45p $
*/

class HDT : public RESPONSE
{
   public:

      HDT();
      virtual ~HDT();

      /*
      ** Data
      */

      double DegreesTrue;

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

      virtual const HDT& operator = ( const HDT& source );
};

#endif // HDT_CLASS_HEADER
