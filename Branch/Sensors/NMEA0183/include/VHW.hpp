#if ! defined( VHW_CLASS_HEADER )

#define VHW_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: vhw.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:43p $
*/

class VHW : public RESPONSE
{
   public:

      VHW();
      virtual ~VHW();

      /*
      ** Data
      */

      double DegreesTrue;
      double DegreesMagnetic;
      double Knots;
      double KilometersPerHour;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const VHW& operator = ( const VHW& source );
};

#endif // VHW_CLASS_HEADER
