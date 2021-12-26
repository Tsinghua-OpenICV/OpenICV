#if ! defined( ROT_CLASS_HEADER )

#define ROT_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: rot.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:48p $
*/

class ROT : public RESPONSE
{
   public:

      ROT();
      virtual ~ROT();

      /*
      ** Data
      */

      double           RateOfTurn;
      NMEA0183_BOOLEAN IsDataValid;

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

      virtual const ROT& operator = ( const ROT& source );
};

#endif // ROT_CLASS_HEADER
