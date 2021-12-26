#if ! defined( STN_CLASS_HEADER )

#define STN_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: stn.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:44p $
*/

class STN : public RESPONSE
{
   public:

      STN();
      virtual ~STN();

      /*
      ** Data
      */

      int TalkerIDNumber;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const STN& operator = ( const STN& source );
};

#endif // STN_CLASS_HEADER
