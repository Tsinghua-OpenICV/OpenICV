#if ! defined( MTW_CLASS_HEADER )

#define MTW_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: mtw.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:50p $
*/

class MTW : public RESPONSE
{
   public:

      MTW();
      virtual ~MTW();

      /*
      ** Data
      */

      double  Temperature;
      QString UnitOfMeasurement;

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

      virtual const MTW& operator = ( const MTW& source );
};

#endif // MTW_CLASS_HEADER
