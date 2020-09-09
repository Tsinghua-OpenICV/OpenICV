#if ! defined( RMC_CLASS_HEADER )

#define RMC_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: rmc.hpp $
** $Revision: 5 $
** $Modtime: 10/12/98 6:44a $
*/

class RMC : public RESPONSE
{
   public:

      RMC();
      virtual ~RMC();

      /*
      ** Data
      */

      QString          UTCTime;
      QDateTime        Time;
      NMEA0183_BOOLEAN IsDataValid;
      LATLONG          Position;
      double           SpeedOverGroundKnots;
      double           TrackMadeGoodDegreesTrue;
      QString          Date;
      double           MagneticVariation;
      EASTWEST         MagneticVariationDirection;
      QString          ModeIndication; 

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual QString PlainEnglish( void ) const;
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const RMC& operator = ( const RMC& source );
};

#endif // RMC_CLASS_HEADER
