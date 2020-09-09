#if ! defined( XTR_CLASS_HEADER )

#define XTR_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: xtr.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:41p $
*/

class XTR : public RESPONSE
{
   public:

      XTR();
      virtual ~XTR();

      /*
      ** Data
      */

      double    Magnitude;
      LEFTRIGHT DirectionToSteer;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const XTR& operator = ( const XTR& source );
};

#endif // XTR_CLASS_HEADER
