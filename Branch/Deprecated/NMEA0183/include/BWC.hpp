#if ! defined( BWC_CLASS_HEADER )

#define BWC_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: bwc.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:39p $
*/

class BWC : public RESPONSE
{
   public:

      BWC();
      virtual ~BWC();

      /*
      ** Data
      */

      QString UTCTime;
      QDateTime   Time;
      double  BearingTrue;
      double  BearingMagnetic;
      double  NauticalMiles;
      QString To;
      LATLONG Position;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      const BWC& operator = ( const BWC& source );
};

#endif // BWC_CLASS_HEADER
