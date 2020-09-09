#if ! defined( TIME__CLASS_HEADER )

#define TIME__CLASS_HEADER

#include "qstring.h"
#include "NMEA0183_slim.h"
#include "Sentence.hpp"
#include "Response.hpp"

class TIME_ : public RESPONSE
{
   public:

      TIME_();
      virtual ~TIME_();

      /*
      ** Data
      */

      QString   UTCTime;
	  QDateTime   Time;

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

      virtual const TIME_& operator = ( const TIME_& source );
};

#endif // TIME__CLASS_HEADER
