#if ! defined( LORAN_TIME_DIFFERENCE_CLASS_HEADER )

#define LORAN_TIME_DIFFERENCE_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: lorantd.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 10:06a $
*/

typedef enum
{
   LoranUnknown = 0,
   LoranValid,
   LoranBlinkWarning,
   LoranCycleWarning,
   LoranSignalToNoiseRatioWarning
}
LORAN_SIGNAL_STATUS;

class LORAN_TIME_DIFFERENCE
{
   public:

      LORAN_TIME_DIFFERENCE();
      virtual ~LORAN_TIME_DIFFERENCE();

      /*
      ** Data
      */

      double              Microseconds;
      LORAN_SIGNAL_STATUS SignalStatus;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual void Parse( int field_number, const SENTENCE& sentence );
      virtual void Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const LORAN_TIME_DIFFERENCE& operator = ( const LORAN_TIME_DIFFERENCE& source );
};


#endif // LORAN_TIME_DIFFERENCE_CLASS_HEADER 
