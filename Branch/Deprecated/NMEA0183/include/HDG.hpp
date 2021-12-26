#if ! defined( HDG_CLASS_HEADER )

#define HDG_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: hdg.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:49p $
*/

class HDG : public RESPONSE
{
   public:

      HDG();
      virtual ~HDG();

      /*
      ** Data
      */

      double   MagneticSensorHeadingDegrees;
      double   MagneticDeviationDegrees;
      EASTWEST MagneticDeviationDirection;
      double   MagneticVariationDegrees;
      EASTWEST MagneticVariationDirection;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const HDG& operator = ( const HDG& source );
};

#endif // HDG_CLASS_HEADER
