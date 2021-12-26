#if ! defined( RESPONSE_CLASS_HEADER )

#define RESPONSE_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** 05 Jun 96 - With Jan-Erik Eriksson's help, added DataSource
**
** $Workfile: response.hpp $
** $Revision: 6 $
** $Modtime: 10/10/98 2:51p $
*/

class NMEA0183;

class RESPONSE
{
   private:

      NMEA0183 *container_p;

   public:

      RESPONSE();
      virtual ~RESPONSE();

      /*
      ** Data
      */

      QString DataSource;
      QString ErrorMessage;
      QString Mnemonic;
      QString Talker;

      /*
      ** Methods
      */

      virtual void Empty( void ) = 0;
      virtual bool Parse( const SENTENCE& sentence ) = 0;
      virtual QString PlainEnglish( void ) const;
      virtual void SetErrorMessage( const QString& );
      virtual void SetContainer( NMEA0183 *container );
      virtual bool Write( SENTENCE& sentence );
};
 
#endif // RESPONSE_CLASS_HEADER
