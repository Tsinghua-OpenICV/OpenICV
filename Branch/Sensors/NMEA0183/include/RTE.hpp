#if ! defined( RTE_CLASS_HEADER )

#define RTE_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: rte.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:48p $
*/

class RTE : public RESPONSE
{
   private:

      void m_DeleteAllEntries( void );

      double m_TotalNumberOfMessages;
      double m_LastMessageNumberReceived;
      double m_MessageNumber;

      int m_LastWaypointNumberWritten;

   public:

      RTE();
      virtual ~RTE();

      typedef enum
      {
         RouteUnknown = 0,
         CompleteRoute,
         WorkingRoute
      }
      ROUTE_TYPE;

      ROUTE_TYPE TypeOfRoute;
      QString    RouteName;
      QVector<QString*>  Waypoints; // Yes, I tried using CStringArray but it is not an array of CStrings, it is an array of LPCSTRs

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );
};

#endif // RTE_CLASS_HEADER
