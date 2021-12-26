#if ! defined( XTE_CLASS_HEADER )

#define XTE_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: xte.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:43p $
*/

class XTE : public RESPONSE
{
   public:

      XTE();
      virtual ~XTE();

      /*
      ** Data
      */

      NMEA0183_BOOLEAN IsLoranBlinkOK;
      NMEA0183_BOOLEAN IsLoranCCycleLockOK;
      double           CrossTrackErrorMagnitude;
      LEFTRIGHT        DirectionToSteer;
      QString          CrossTrackUnits;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const XTE& operator = ( const XTE& source );
};

#endif // XTE_CLASS_HEADER
