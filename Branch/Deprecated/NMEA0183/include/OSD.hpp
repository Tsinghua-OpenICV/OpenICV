#if ! defined( OSD_CLASS_HEADER )

#define OSD_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: osd.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:46p $
*/

class OSD : public RESPONSE
{
   public:

      OSD();
      virtual ~OSD();

      /*
      ** Data
      */

      double           HeadingDegreesTrue;
      NMEA0183_BOOLEAN IsHeadingValid;
      double           VesselCourseDegreesTrue;
      REFERENCE        VesselCourseReference;
      double           VesselSpeed;
      REFERENCE        VesselSpeedReference;
      double           VesselSetDegreesTrue;
      double           VesselDriftSpeed;
      QString          VesselDriftSpeedUnits;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const OSD& operator = ( const OSD& source );
};

#endif // OSD_CLASS_HEADER
