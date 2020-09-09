#if ! defined( ALM_CLASS_HEADER )

#define ALM_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: alm.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:49p $
*/

class ALM : public RESPONSE
{
   public:

      ALM();
      virtual ~ALM();

      /*
      ** Data
      */

      WORD  NumberOfMessages;
      WORD  MessageNumber;
      WORD  PRNNumber;
      WORD  WeekNumber;
      WORD  SVHealth;
      WORD  Eccentricity;
      WORD  AlmanacReferenceTime;
      WORD  InclinationAngle;
      WORD  RateOfRightAscension;
      unsigned int RootOfSemiMajorAxis;
      unsigned int ArgumentOfPerigee;
      unsigned int LongitudeOfAscensionNode;
      unsigned int MeanAnomaly;
      WORD  F0ClockParameter;
      WORD  F1ClockParameter;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      const ALM& operator = ( const ALM& source );
};

#endif // ALM_CLASS_HEADER
