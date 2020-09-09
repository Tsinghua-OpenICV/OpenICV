#if ! defined( FSI_CLASS_HEADER )

#define FSI_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: fsi.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:39p $
*/

class FSI : public RESPONSE
{
   public:

      FSI();
      virtual ~FSI();

      /*
      ** Data
      */

      double              TransmittingFrequency;
      double              ReceivingFrequency;
      COMMUNICATIONS_MODE Mode;
      short               PowerLevel;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const FSI& operator = ( const FSI& source );
};

#endif // FSI_CLASS_HEADER
