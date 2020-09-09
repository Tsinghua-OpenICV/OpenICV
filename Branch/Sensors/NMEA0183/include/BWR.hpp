#if ! defined( BWR_CLASS_HEADER )

#define BWR_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: bwr.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:49p $
*/

class BWR : public RESPONSE
{
   public:

      BWR();
      virtual ~BWR();

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

      virtual const BWR& operator = ( const BWR& source );
};

#endif // BWR_CLASS_HEADER
