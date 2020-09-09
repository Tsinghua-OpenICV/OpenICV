#if ! defined( RPM_CLASS_HEADER )

#define RPM_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: rpm.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:44p $
*/

class RPM : public RESPONSE
{
   public:

      RPM();
      virtual ~RPM();

      /*
      ** Data
      */

      QString          Source;
      int              SourceNumber;
      double           RevolutionsPerMinute;
      double           PropellerPitchPercentage;
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

      virtual const RPM& operator = ( const RPM& source );
};

#endif // RPM_CLASS_HEADER
