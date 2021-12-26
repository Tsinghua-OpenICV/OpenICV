#if ! defined( OMEGA_PAIR_CLASS_HEADER )

#define OMEGA_PAIR_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: omegapar.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 10:05a $
*/

class OMEGA_PAIR
{
   public:

      OMEGA_PAIR();
      virtual ~OMEGA_PAIR();

      /*
      ** Data
      */

      QString Name;
      int     LaneNumber;
      int     CentilaneNumber;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual void Parse( int field_number, const SENTENCE& sentence );
      virtual void Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const OMEGA_PAIR& operator = ( const OMEGA_PAIR& source );
};

#endif // OMEGA_PAIR_CLASS_HEADER
