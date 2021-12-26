#if ! defined( RSD_CLASS_HEADER )

#define RSD_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: rsd.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:44p $
*/

typedef enum
{
   RotationUnknown = 0,
   CourseUpRotation,
   HeadUpRotation,
   NorthUpRotation
}
ROTATION;

class RSD : public RESPONSE
{
   public:

      RSD();
      virtual ~RSD();

      /*
      ** Data
      */

      RADAR_DATA Data1;
      RADAR_DATA Data2;
      double     CursorRangeFromOwnShip;
      double     CursorBearingDegreesClockwiseFromZero;
      double     RangeScale;
      QString    RangeUnits;
      ROTATION   DisplayRotation;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const RSD& operator = ( const RSD& source );
};

#endif // RSD_CLASS_HEADER
