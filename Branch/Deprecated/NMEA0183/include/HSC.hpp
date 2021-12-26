#if ! defined( HSC_CLASS_HEADER )

#define HSC_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: hsc.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:45p $
*/

class HSC : public RESPONSE
{
   public:

      HSC();
      virtual ~HSC();

      /*
      ** Data
      */

      double DegreesMagnetic;
      double DegreesTrue;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const HSC& operator = ( const HSC& source );
};

#endif // HSC_CLASS_HEADER
