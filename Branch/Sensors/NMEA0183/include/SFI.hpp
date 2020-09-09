#if ! defined( SFI_CLASS_HEADER )

#define SFI_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: sfi.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:44p $
*/

class SFI : public RESPONSE
{
   public:

      SFI();
      virtual ~SFI();

      /*
      ** Data
      */
       
      double             TotalMessages;
      double             MessageNumber;
      FREQUENCY_AND_MODE Frequencies[ 6 ];

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const SFI& operator = ( const SFI& source );
};

#endif // SFI_CLASS_HEADER
