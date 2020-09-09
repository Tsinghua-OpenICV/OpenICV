#if ! defined( RADAR_DATA_CLASS_HEADER )

#define RADAR_DATA_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: radardat.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 10:05a $
*/

class RADAR_DATA
{
   public:

      RADAR_DATA();
      virtual ~RADAR_DATA();

      /*
      ** Data
      */

      double OriginRangeFromOwnShip;
      double OriginBearingDegreesFromZero;
      double VariableRangeMarkerRange;
      double BearingLineDegreesFromZero;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual void Parse( int field_number, const SENTENCE& sentence );
      virtual void Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const RADAR_DATA& operator = ( const RADAR_DATA& source );
};

#endif // RADAR_DATA_CLASS_HEADER
