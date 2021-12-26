#if ! defined( SATELLITE_DATA_CLASS_HEADER )

#define SATELLITE_DATA_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: satdat.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 10:05a $
*/

class SATELLITE_DATA
{
   public:

      SATELLITE_DATA();
      virtual ~SATELLITE_DATA();

      /*
      ** Data
      */

      int SatelliteNumber;
      int ElevationDegrees;
      int AzimuthDegreesTrue;
      int SignalToNoiseRatio;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual void Parse( int field_number, const SENTENCE& sentence );
      virtual void Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const SATELLITE_DATA& operator = ( const SATELLITE_DATA& source );
};

#endif // SATELLITE_DATA_CLASS_HEADER
