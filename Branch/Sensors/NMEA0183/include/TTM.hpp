#if ! defined( TTM_CLASS_HEADER )

#define TTM_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: ttm.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:44p $
*/

typedef enum
{
   TargetUnknown = 0,
   TargetLost,
   TargetQuery,
   TargetTracking
}
TARGET_STATUS;

class TTM : public RESPONSE
{
   public:

      TTM();
      virtual ~TTM();

      /*
      ** Data
      */

      int           TargetNumber;
      double        TargetDistance;
      double        BearingFromOwnShip;
      QString       BearingUnits;
      double        TargetSpeed;
      double        TargetCourse;
      QString       TargetCourseUnits;
      double        DistanceOfClosestPointOfApproach;
      double        NumberOfMinutesToClosestPointOfApproach;
      QString       Increasing;
      QString       TargetName;
      TARGET_STATUS TargetStatus;
      QString       ReferenceTarget;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const TTM& operator = ( const TTM& source );
};

#endif // TTM_CLASS_HEADER
