#if ! defined( MWV_CLASS_HEADER )

#define MWV_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: mwv.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:45p $
*/

class MWV : public RESPONSE
{
   public:

      MWV();
      virtual ~MWV();

      /*
      ** Data
      */

      double           WindAngle;
      QString          Reference;
      double           WindSpeed;
      QString          WindSpeedUnits;
      NMEA0183_BOOLEAN IsDataValid;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const MWV& operator = ( const MWV& source );
};

#endif // MWV_CLASS_HEADER
