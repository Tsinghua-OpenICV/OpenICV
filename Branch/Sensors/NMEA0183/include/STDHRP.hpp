#if ! defined( STDHRP_CLASS_HEADER )

#define STDHRP_CLASS_HEADER

#include "qstring.h"
#include "NMEA0183_slim.h"
#include "LatLong.hpp"
#include "Sentence.hpp"
#include "Response.hpp"

class STDHRP : public RESPONSE
{
   public:

      STDHRP();
      virtual ~STDHRP();

      /*
      ** Data
      */

      double stdHeading;
	  double stdRoll;
	  double stdPitch;

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

      virtual const STDHRP& operator = ( const STDHRP& source );
};

#endif // STDHRP_CLASS_HEADER
