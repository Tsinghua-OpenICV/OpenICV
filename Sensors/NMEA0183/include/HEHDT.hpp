#if ! defined( HEHDT_CLASS_HEADER )

#define HEHDT_CLASS_HEADER

#include "qstring.h"
#include "NMEA0183_slim.h"
#include "LatLong.hpp"
#include "Sentence.hpp"
#include "Response.hpp"

class HEHDT : public RESPONSE
{
   public:

      HEHDT();
      virtual ~HEHDT();

      /*
      ** Data
      */

      double heading;

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

      virtual const HEHDT& operator = ( const HEHDT& source );
};

#endif // HEHDT_CLASS_HEADER
