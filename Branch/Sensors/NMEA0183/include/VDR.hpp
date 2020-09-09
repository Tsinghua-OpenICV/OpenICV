#if ! defined( VDR_CLASS_HEADER )

#define VDR_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: vdr.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:44p $
*/

class VDR : public RESPONSE
{
   public:

      VDR();
      virtual ~VDR();

      /*
      ** Data
      */

      double DegreesTrue;
      double DegreesMagnetic;
      double Knots;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const VDR& operator = ( const VDR& source );
};

#endif // VDR_CLASS_HEADER
