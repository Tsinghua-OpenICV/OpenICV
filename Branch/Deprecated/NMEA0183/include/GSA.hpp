#if ! defined( GSA_CLASS_HEADER )

#define GSA_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: gsa.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:49p $
*/

class GSA : public RESPONSE
{
   public:

      GSA();
      virtual ~GSA();

      /*
      ** Data
      */

      typedef enum
      {
         UnknownOperatingMode = 0,
         Manual,
         Automatic
      }
      OPERATING_MODE;

      typedef enum
      {
         FixUnknown = 0,
         FixUnavailable,
         TwoDimensional,
         ThreeDimensional
      }
      FIX_MODE;

      OPERATING_MODE OperatingMode;
      FIX_MODE       FixMode;
      int            SatelliteNumber[ 12 ];
      double         PDOP;
      double         HDOP;
      double         VDOP;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const GSA& operator = ( const GSA& source );
};

#endif // GSA_CLASS_HEADER
