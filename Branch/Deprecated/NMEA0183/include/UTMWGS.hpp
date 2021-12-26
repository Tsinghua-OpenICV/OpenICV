#if ! defined( UTMWGS_CLASS_HEADER )

#define UTMWGS_CLASS_HEADER

#include "qstring.h"
#include "NMEA0183_slim.h"
#include "Sentence.hpp"
#include "Response.hpp"

class UTMWGS : public RESPONSE
{
   public:

      UTMWGS();
      virtual ~UTMWGS();

      /*
      ** Data
      */

	  QChar UTMzoneChar ; //
	  int  UTMzone;
		double east;
		double north;
		double up;

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

      virtual const UTMWGS& operator = ( const UTMWGS& source );
};

#endif // UTMWGS_CLASS_HEADER
