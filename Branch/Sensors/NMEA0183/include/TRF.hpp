#if ! defined( TRF_CLASS_HEADER )

#define TRF_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: trf.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:50p $
*/

class TRF : public RESPONSE
{
   public:

      TRF();
      virtual ~TRF();

      /*
      ** Data
      */

      QString          UTCTime;
      QDateTime        Time;
      QString          Date;
      LATLONG          Position;
      double           ElevationAngle;
      double           NumberOfIterations;
      double           NumberOfDopplerIntervals;
      double           UpdateDistanceNauticalMiles;
      double           SatelliteID;
      NMEA0183_BOOLEAN IsDataValid;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const TRF& operator = ( const TRF& source );
};

#endif // TRF_CLASS_HEADER
