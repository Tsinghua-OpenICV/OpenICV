#if ! defined( MTA_CLASS_HEADER )

#define MTA_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: mta.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:45p $
*/

/*
** This Sentence Not Recommended For New Designs
** ZDA is recommended.
*/

class MTA : public RESPONSE
{
   public:

      MTA();
      virtual ~MTA();

      /*
      ** Data
      */

      double  Temperature;
      QString UnitOfMeasurement;

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

      virtual const MTA& operator = ( const MTA& source );
};

#endif // MTA_CLASS_HEADER
