#if ! defined( RSA_CLASS_HEADER )

#define RSA_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: rsa.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:45p $
*/

class RSA : public RESPONSE
{
   public:

      RSA();
      virtual ~RSA();

      /*
      ** Data
      */

      double           Starboard;
      NMEA0183_BOOLEAN IsStarboardDataValid;
      double           Port;
      NMEA0183_BOOLEAN IsPortDataValid;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const RSA& operator = ( const RSA& source );
};

#endif // RSA_CLASS_HEADER
