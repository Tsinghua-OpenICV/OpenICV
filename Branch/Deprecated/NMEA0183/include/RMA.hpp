#if ! defined( RMA_CLASS_HEADER )

#define RMA_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: rma.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:48p $
*/

class RMA : public RESPONSE
{
   public:

      RMA();
      virtual ~RMA();

      /*
      ** Data
      */

      NMEA0183_BOOLEAN IsDataValid;
      LATLONG          Position;
      double           TimeDifferenceA;
      double           TimeDifferenceB;
      double           SpeedOverGroundKnots;
      double           TrackMadeGoodDegreesTrue;
      double           MagneticVariation;
      EASTWEST         MagneticVariationDirection;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const RMA& operator = ( const RMA& source );
};

#endif // RMA_CLASS_HEADER
