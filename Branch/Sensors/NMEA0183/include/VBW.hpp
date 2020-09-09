#if ! defined( VBW_CLASS_HEADER )

#define VBW_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: vbw.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:44p $
*/

class VBW : public RESPONSE
{
   public:

      VBW();
      virtual ~VBW();

      /*
      ** Data
      */

      double           LongitudinalWaterSpeed;
      double           TransverseWaterSpeed;
      NMEA0183_BOOLEAN IsWaterSpeedValid;
      double           LongitudinalGroundSpeed;
      double           TransverseGroundSpeed;
      NMEA0183_BOOLEAN IsGroundSpeedValid;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const VBW& operator = ( const VBW& source );
};

#endif // VBW_CLASS_HEADER
