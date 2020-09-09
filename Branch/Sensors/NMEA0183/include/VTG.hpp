#if ! defined( VTG_CLASS_HEADER )

#define VTG_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: vtg.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:43p $
*/

class VTG : public RESPONSE
{
   public:

      VTG();
      virtual ~VTG();

      /*
      ** Data
      */

       double TrackDegreesTrue;
       double TrackDegreesMagnetic;
	   double SpeedKnots;
	   double SpeedKilometersPerHour;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const VTG& operator = ( const VTG& source );
};

#endif // VTG_CLASS_HEADER
