#if ! defined( LCD_CLASS_HEADER )

#define LCD_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: lcd.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:50p $
*/

class LCD : public RESPONSE
{
   public:

      LCD();
      virtual ~LCD();

      /*
      ** Data
      */

      int             GroupRepetitionInterval;
      RATIO_AND_PULSE Master;
      RATIO_AND_PULSE Secondary1;
      RATIO_AND_PULSE Secondary2;
      RATIO_AND_PULSE Secondary3;
      RATIO_AND_PULSE Secondary4;
      RATIO_AND_PULSE Secondary5;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const LCD& operator = ( const LCD& source );
};

#endif // LCD_CLASS_HEADER
