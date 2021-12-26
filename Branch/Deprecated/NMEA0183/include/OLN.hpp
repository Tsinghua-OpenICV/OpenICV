#if ! defined( OLN_CLASS_HEADER )

#define OLN_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: oln.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:48p $
*/

class OLN : public RESPONSE
{
   public:

      OLN();
      virtual ~OLN();

      /*
      ** Data
      */

      OMEGA_PAIR Pair1;
      OMEGA_PAIR Pair2;
      OMEGA_PAIR Pair3;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const OLN& operator = ( const OLN& source );
};

#endif // OLN_CLASS_HEADER
