#if ! defined( MANUFACTURER_CLASS_HEADER )

#define MANUFACTURER_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1997, Samuel R. Blackburn
**
** $Workfile: manufact.hpp $
** $Revision: 5 $
** $Modtime: 10/10/98 3:16p $
*/

class MANUFACTURER
{
   public:

      MANUFACTURER();
      MANUFACTURER( const char *mnemonic, const char *company_name );
      virtual ~MANUFACTURER();

      QString Mnemonic;
      QString CompanyName;
};

#endif // MANUFACTURER_CLASS_HEADER    
