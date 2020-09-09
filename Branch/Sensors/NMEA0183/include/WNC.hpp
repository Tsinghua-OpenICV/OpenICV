#if ! defined( WNC_CLASS_HEADER )

#define WNC_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: wnc.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:43p $
*/

class WNC : public RESPONSE
{
   public:

      WNC();
      virtual ~WNC();

      /*
      ** Data
      */

      double  MilesDistance;
      double  KilometersDistance;
      QString To;
      QString From;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const WNC& operator = ( const WNC& source );
};

#endif // WNC_CLASS_HEADER
