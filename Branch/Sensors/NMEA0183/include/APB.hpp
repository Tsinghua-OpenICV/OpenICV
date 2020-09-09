#if ! defined( APB_CLASS_HEADER )

#define APB_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: apb.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:38p $
*/

class APB : public RESPONSE
{
   public:

      APB();
      virtual ~APB();

      /*
      ** Data
      */

      NMEA0183_BOOLEAN IsLoranBlinkOK;
      NMEA0183_BOOLEAN IsLoranCCycleLockOK;
      double           CrossTrackErrorMagnitude;
      LEFTRIGHT        DirectionToSteer;
      QString          CrossTrackUnits;
      NMEA0183_BOOLEAN IsArrivalCircleEntered;
      NMEA0183_BOOLEAN IsPerpendicular;
      double           BearingOriginToDestination;
      QString          BearingOriginToDestinationUnits;
      QString          To;
      double           BearingPresentPositionToDestination;
      QString          BearingPresentPositionToDestinationUnits;
      double           HeadingToSteer;
      QString          HeadingToSteerUnits;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const APB& operator = ( const APB& source );
};

#endif // APB_CLASS_HEADER
