#if ! defined( STDSPD_CLASS_HEADER )

#define STDSPD_CLASS_HEADER

#include "qstring.h"
#include "NMEA0183_slim.h"
#include "Sentence.hpp"
#include "Response.hpp"

class STDSPD : public RESPONSE
{
   public:

      STDSPD();
      virtual ~STDSPD();

      /*
      ** Data
      */

      
    double stdHdg;
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

      virtual const STDSPD& operator = ( const STDSPD& source );
};

#endif // STDSPD_CLASS_HEADER
