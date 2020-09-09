#if ! defined( ZDA_CLASS_HEADER )

#define ZDA_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: zda.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:41p $
*/

class ZDA : public RESPONSE
{
   public:

      ZDA();
      virtual ~ZDA();

      /*
      ** Data
      */

      QString	UTCTime;
      QDateTime Time;
      int		Day;
      int		Month;
      int		Year;
      int		LocalHourDeviation;
      int		LocalMinutesDeviation;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const ZDA& operator = ( const ZDA& source );
};

#endif // ZDA_CLASS_HEADER
