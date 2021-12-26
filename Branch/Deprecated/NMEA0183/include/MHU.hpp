#if ! defined( MHU_CLASS_HEADER )

#define MHU_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: mhu.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:50p $
*/

/*
** This Sentence Not Recommended For New Designs
** XDR is recommended.
*/

class MHU : public RESPONSE
{
   public:

      MHU();
      virtual ~MHU();

      /*
      ** Data
      */

      double RelativeHumidityPercent;
      double AbsoluteHumidityPercent;
      double DewPointDegreesCelcius;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const MHU& operator = ( const MHU& source );
};

#endif // MHU_CLASS_HEADER
