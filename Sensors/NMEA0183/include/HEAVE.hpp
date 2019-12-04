#if ! defined( HEAVE_CLASS_HEADER )

#define HEAVE_CLASS_HEADER

#include "qstring.h"
#include "NMEA0183_slim.h"
#include "LatLong.hpp"
#include "Sentence.hpp"
#include "Response.hpp"

class HEAVE : public RESPONSE
{
   public:

      HEAVE();
      virtual ~HEAVE();

      /*
      ** Data
      */

      double surge;
	  double sway;
	  double heave;

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

      virtual const HEAVE& operator = ( const HEAVE& source );
};

#endif // HEAVE_CLASS_HEADER
