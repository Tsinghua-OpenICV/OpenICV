#if ! defined( GLC_CLASS_HEADER )

#define GLC_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: glc.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:49p $
*/

class GLC : public RESPONSE
{
   public:

      GLC();
      virtual ~GLC();

      /*
      ** Data
      */
       
      int                   GroupRepetitionInterval;
      LORAN_TIME_DIFFERENCE MasterTOA;
      LORAN_TIME_DIFFERENCE TimeDifference1;
      LORAN_TIME_DIFFERENCE TimeDifference2;
      LORAN_TIME_DIFFERENCE TimeDifference3;
      LORAN_TIME_DIFFERENCE TimeDifference4;
      LORAN_TIME_DIFFERENCE TimeDifference5;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const GLC& operator = ( const GLC& source );
};

#endif // GLC_CLASS_HEADER
