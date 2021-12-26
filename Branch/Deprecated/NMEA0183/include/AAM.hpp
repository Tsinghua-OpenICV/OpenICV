#if ! defined( AAM_CLASS_HEADER )

#define AAM_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: aam.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:38p $
*/

class AAM : public RESPONSE
{
   public:

      AAM();
      virtual ~AAM();

      /*
      ** Data
      */

      NMEA0183_BOOLEAN IsArrivalCircleEntered;
      NMEA0183_BOOLEAN IsPerpendicular;
      double           CircleRadius;
      QString          WaypointID;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const AAM& operator = ( const AAM& source );
};

#endif // AAM_CLASS_HEADER
