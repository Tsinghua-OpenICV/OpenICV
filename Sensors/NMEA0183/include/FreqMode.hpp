#if ! defined( FREQUENCY_AND_MODE_CLASS_HEADER )

#define FREQUENCY_AND_MODE_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: freqmode.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 10:07a $
*/

class FREQUENCY_AND_MODE
{
   public:

      FREQUENCY_AND_MODE();
      virtual ~FREQUENCY_AND_MODE();

      /*
      ** Data
      */

      double              Frequency;
      COMMUNICATIONS_MODE Mode;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual void Parse( int field_number, const SENTENCE& sentence );
      virtual void Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const FREQUENCY_AND_MODE& operator = ( const FREQUENCY_AND_MODE& source );
};

#endif // FREQUENCY_AND_MODE_CLASS_HEADER
