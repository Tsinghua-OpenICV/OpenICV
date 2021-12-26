#if ! defined( GXA_CLASS_HEADER )

#define GXA_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: gxa.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:48p $
*/

class GXA : public RESPONSE
{
   public:

      GXA();
      virtual ~GXA();

      /*
      ** Data
      */

      QString UTCTime;
      QDateTime   Time;
      LATLONG Position;
      QString WaypointID;
      WORD    SatelliteNumber;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const GXA& operator = ( const GXA& source );
};

#endif // GXA_CLASS_HEADER
