#if ! defined( LINE_OF_POSITION_CLASS_HEADER )

#define LINE_OF_POSITION_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: deccalop.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 10:07a $
*/

class LINE_OF_POSITION
{
   public:

      LINE_OF_POSITION();
      virtual ~LINE_OF_POSITION();

      /*
      ** Data
      */

      QString          ZoneID;
      double           LineOfPosition;
      NMEA0183_BOOLEAN MasterLine;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual void Parse( int field_number, const SENTENCE& sentence );
      virtual void Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const LINE_OF_POSITION& operator = ( const LINE_OF_POSITION& source );
};

#endif // LINE_OF_POSITION_CLASS_HEADER
