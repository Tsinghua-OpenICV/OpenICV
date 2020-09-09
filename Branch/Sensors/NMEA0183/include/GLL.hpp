#if ! defined( GLL_CLASS_HEADER )

#define GLL_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: gll.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:49p $
*/

class GLL : public RESPONSE
{
   public:

      GLL();
      virtual ~GLL();

      /*
      ** Data
      */

      QString          UTCTime;
      QDateTime        Time;
      NMEA0183_BOOLEAN IsDataValid;
      LATLONG          Position;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual QString PlainEnglish( void ) const;
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const GLL& operator = ( const GLL& source );
};

#endif // GLL_CLASS_HEADER
