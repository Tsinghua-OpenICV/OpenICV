#if ! defined( BEC_CLASS_HEADER )

#define BEC_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: bec.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:38p $
*/

class BEC : public RESPONSE
{
   public:

      BEC();
      virtual ~BEC();

      /*
      ** Data
      */

      QString UTCTime;
      QDateTime   Time;
      LATLONG Position;
      double  BearingTrue;
      double  BearingMagnetic;
      double  DistanceNauticalMiles;
      QString To;

      /*
      ** Methods
      */
       
      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      const BEC& operator = ( const BEC& source );
};

#endif // BEC_CLASS_HEADER
