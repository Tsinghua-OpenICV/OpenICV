#if ! defined( RATIO_AND_PULSE_CLASS_HEADER )

#define RATIO_AND_PULSE_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: ratiopls.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 10:05a $
*/

class RATIO_AND_PULSE
{
   public:

      RATIO_AND_PULSE();
      virtual ~RATIO_AND_PULSE();

      /*
      ** Data
      */

      int SignalToNoiseRatio;
      int PulseShapeECD;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual void Parse( int field_number, const SENTENCE& sentence );
      virtual void Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const RATIO_AND_PULSE& operator = ( const RATIO_AND_PULSE& source );
};

#endif // RATIO_AND_PULSE_CLASS_HEADER
