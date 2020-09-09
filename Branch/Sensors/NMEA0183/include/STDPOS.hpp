#if ! defined( STDPOS_CLASS_HEADER )

#define STDPOS_CLASS_HEADER

#include "qstring.h"
#include "NMEA0183_slim.h"
#include "LatLong.hpp"
#include "Sentence.hpp"
#include "Response.hpp"

class STDPOS : public RESPONSE
{
   public:

      STDPOS();
      virtual ~STDPOS();

      /*
      ** Data
      */

      double stdLat;
	  double stdLong;
	  double stdAlt;

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

      virtual const STDPOS& operator = ( const STDPOS& source );
};

#endif // STDPOS_CLASS_HEADER
