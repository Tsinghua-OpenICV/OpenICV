#if ! defined( P_CLASS_HEADER )

#define P_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: p.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:45p $
*/

class P : public RESPONSE
{
   public:

      P();
      virtual ~P();

      /*
      ** Data
      */

      MANUFACTURER_LIST Manufacturers;

      QString Sentence;
      QString Source;
      QString CompanyName;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const P& operator = ( const P& source );
};

#endif // P_CLASS_HEADER
