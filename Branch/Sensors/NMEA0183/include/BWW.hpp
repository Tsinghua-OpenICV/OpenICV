#if ! defined( BWW_CLASS_HEADER )

#define BWW_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: bww.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:38p $
*/

class BWW : public RESPONSE
{
   public:

      BWW();
      virtual ~BWW();

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
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const BWW& operator = ( const BWW& source );
};

#endif // BWW_CLASS_HEADER
