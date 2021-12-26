#if ! defined( VLW_CLASS_HEADER )

#define VLW_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: vlw.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:48p $
*/

class VLW : public RESPONSE
{
   public:

      VLW();
      virtual ~VLW();

      /*
      ** Data
      */

      double TotalDistanceNauticalMiles;
      double DistanceSinceResetNauticalMiles;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const VLW& operator = ( const VLW& source );
};

#endif // VLW_CLASS_HEADER
