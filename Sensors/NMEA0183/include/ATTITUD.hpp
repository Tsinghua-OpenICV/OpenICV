#if ! defined( ATTITUD_CLASS_HEADER )

#define ATTITUD_CLASS_HEADER

#include "qstring.h"
#include "NMEA0183_slim.h"
#include "LatLong.hpp"
#include "Sentence.hpp"
#include "Response.hpp"

class ATTITUD : public RESPONSE
{
   public:

      ATTITUD();
      virtual ~ATTITUD();

      /*
      ** Data
      */

	  double roll;
      double pitch;
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

      virtual const ATTITUD& operator = ( const ATTITUD& source );
};

#endif // ATTITUD_CLASS_HEADER
