#if ! defined( HSPOS_CLASS_HEADER )

#define HSPOS_CLASS_HEADER

#include "qstring.h"
#include "NMEA0183_slim.h"
#include "LatLong.hpp"
#include "Sentence.hpp"
#include "Response.hpp"

class HSPOS : public RESPONSE
{
   public:

      HSPOS();
      virtual ~HSPOS();

      /*
      ** Data
      */

      QString   UTCTime;
      QDateTime Time;
      LATLONG	Position;
      double depth;
	  double altitude;
	  double latitudeStdDev; //meters
	  double longitudeStdDev; //meters
	  double latLongCov; //meters
	  double depthStdDev;
	  int UTMzone; //
	  QChar UTMzoneChar ; //
	  double eastindProjection; //
	  double northingProjection; //
	  double logMislignment; //degree
	  double logScaleFactorError; //%
	  double compensationSoundVelocity; //%m/s

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

      virtual const HSPOS& operator = ( const HSPOS& source );
};

#endif // HSPOS_CLASS_HEADER
