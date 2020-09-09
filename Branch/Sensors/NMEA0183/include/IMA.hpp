#if ! defined( IMA_CLASS_HEADER )

#define IMA_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: ima.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:45p $
*/

/*
** This Sentence Not Recommended For New Designs
** There is no recommended replacement.
*/

class IMA : public RESPONSE
{
   public:

      IMA();
      virtual ~IMA();

      /*
      ** Data
      */

      QString  VesselName;
      QString  Callsign;
      LATLONG  Position;
      double   HeadingDegreesTrue;
      double   HeadingDegreesMagnetic;
      double   SpeedKnots;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const IMA& operator = ( const IMA& source );
};

#endif // IMA_CLASS_HEADER
