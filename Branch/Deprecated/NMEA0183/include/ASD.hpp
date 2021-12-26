#if ! defined( ASD_CLASS_HEADER )

#define ASD_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: asd.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:49p $
*/

class ASD : public RESPONSE
{
   public:

      ASD();
      virtual ~ASD();

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );
};

#endif // ASD_CLASS_HEADER
