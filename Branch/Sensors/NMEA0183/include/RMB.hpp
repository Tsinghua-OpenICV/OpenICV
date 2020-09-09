#if ! defined( RMB_CLASS_HEADER )

#define RMB_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: rmb.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:48p $
*/

class RMB : public RESPONSE
{
   public:

      RMB();
      virtual ~RMB();

      /*
      ** Data
      */

      NMEA0183_BOOLEAN IsDataValid;
      double           CrossTrackError;
      LEFTRIGHT        DirectionToSteer;
      QString          To;
      QString          From;
      LATLONG          DestinationPosition;
      double           RangeToDestinationNauticalMiles;
      double           BearingToDestinationDegreesTrue;
      double           DestinationClosingVelocityKnots;
      NMEA0183_BOOLEAN IsArrivalCircleEntered;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const RMB& operator = ( const RMB& source );
};

#endif // RMB_CLASS_HEADER
