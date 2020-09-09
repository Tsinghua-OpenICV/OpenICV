#if ! defined( DPT_CLASS_HEADER )

#define DPT_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: dpt.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:38p $
*/

class DPT : public RESPONSE
{
   public:

      DPT();
      virtual ~DPT();

      /*
      ** Data
      */

      double DepthMeters;
      double OffsetFromTransducerMeters;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const DPT& operator = ( const DPT& source );
};

#endif // DPT_CLASS_HEADER
