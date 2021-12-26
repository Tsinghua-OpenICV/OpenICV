#if ! defined( BOD_CLASS_HEADER )

#define BOD_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: bod.hpp $
** $Revision: 5 $
** $Modtime: 10/10/98 4:39p $
*/

class BOD : public RESPONSE
{
   public:

      BOD();
      virtual ~BOD();

      /*
      ** Data
      */

      double  BearingTrue;
      double  BearingMagnetic;
      QString To;
      QString From;

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

      virtual const BOD& operator = ( const BOD& source );
};

#endif // BOD_CLASS_HEADER
