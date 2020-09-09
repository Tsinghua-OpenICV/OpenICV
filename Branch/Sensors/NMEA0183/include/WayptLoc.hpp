#if ! defined( WAYPOINT_LOCATION_CLASS_HEADER )

#define WAYPOINT_LOCATION_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: wayptloc.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:41p $
*/

/*
** This Sentence Not Recommended For New Designs
** A combination of WPL, GLL, ZDA and ZTG is recommended.
*/

class WAYPOINT_LOCATION : public RESPONSE
{
   public:

      WAYPOINT_LOCATION();
      virtual ~WAYPOINT_LOCATION();

      /*
      ** Data
      */

      QString	UTCTime;
      QDateTime Time;
      LATLONG	Position;
      QString	Waypoint;

      /*
      ** Methods
      */

      virtual void  Empty( void );
      virtual BOOL  Parse( const SENTENCE& sentence );
      virtual QString PlainEnglish( void ) const;
      virtual BOOL  Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const WAYPOINT_LOCATION& operator = ( const WAYPOINT_LOCATION& source );
};

#endif // WAYPOINT_LOCATION_CLASS_HEADER
