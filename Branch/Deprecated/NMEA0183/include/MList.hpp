#if ! defined( MANUFACTURER_LIST_CLASS_HEADER )

#define MANUFACTURER_LIST_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: mlist.hpp $
** $Revision: 5 $
** $Modtime: 10/10/98 4:59p $
*/

class MANUFACTURER_LIST
{
   public:

      MANUFACTURER_LIST();
      virtual ~MANUFACTURER_LIST();

      /*
      ** Data
      */

      QVector<MANUFACTURER*> ManufacturerTable;

      /*
      ** Methods
      */

      virtual void Add( MANUFACTURER *manufacturer_p );
      virtual void Empty( void );
      virtual void Fill( void );
      virtual const QString& Find( const QString& mnemonic );
      virtual void Sort( void );

      /*
      ** Operators
      */

      virtual const QString& operator [] ( const char *mnemonic );
};

#endif // MANUFACTURER_LIST_CLASS_HEADER    
