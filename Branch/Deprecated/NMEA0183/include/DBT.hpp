#if ! defined( DBT_CLASS_HEADER )

#define DBT_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: dbt.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:38p $
*/

class DBT : public RESPONSE
{
   public:

      DBT();
      virtual ~DBT();

      /*
      ** Data
      */

      double DepthFeet;
      double DepthMeters;
      double DepthFathoms;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      const DBT& operator = ( const DBT& source );
};

#endif // DBT_CLASS_HEADER
