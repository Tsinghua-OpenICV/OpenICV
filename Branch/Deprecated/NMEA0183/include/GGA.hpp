#if ! defined( GGA_CLASS_HEADER )

#define GGA_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: gga.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:39p $
*/

class GGA : public RESPONSE
{
   public:

      GGA();
      virtual ~GGA();

      /*
      ** Data
      */

      QString   UTCTime;
      QDateTime Time;
      LATLONG	Position;
      int		GPSQuality;
      int		NumberOfSatellitesInUse;
      double	HorizontalDilutionOfPrecision;
      double	AntennaAltitudeMeters;
      double	GeoidalSeparationMeters;
      double	AgeOfDifferentialGPSDataSeconds;
      int		DifferentialReferenceStationID;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual QString PlainEnglish( void ) const;
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const GGA& operator = ( const GGA& source );
};

#endif // GGA_CLASS_HEADER
