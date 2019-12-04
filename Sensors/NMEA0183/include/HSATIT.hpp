#if ! defined( HSATIT_CLASS_HEADER )

#define HSATIT_CLASS_HEADER

#include "qstring.h"
#include "NMEA0183_slim.h"
#include "nmea0183.h"
#include "LatLong.hpp"
#include "Sentence.hpp"
#include "Response.hpp"

class HSATIT : public RESPONSE
{
   public:

      HSATIT();
      virtual ~HSATIT();

    /*
    ** Data
    */
	
    double heading;	//deg
	double roll;	//deg
	double pitch;	//deg
	double heave;	//meters
	double yawRotationRate;		// deg.s-1
	double rollRotationRate;	// deg.s-1
	double pitchRotationRate;	// deg.s-1
	double courseMadeGood;		// deg
	double speedOverGround;		//
	double longitudinalVel;		// m.s-1 (positive toward the box)
	double transveralVel;		// m.s-1 (positive toward the port side)
	double verticalVel;			// m.s-1 (positive toward upside)
	double headingStdDev;		// deg
	double rollStdDev;			// deg
	double pitchStdDev;			// deg
	double northSpeedStdDev;	// m.s-1
	double eastSpeeedStdDev;	// m.s-1
	double verticalStdDev;		// m.s-1

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual QString PlainEnglish( void ) const;
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const HSATIT& operator = ( const HSATIT& source );
};

#endif // HSATIT_CLASS_HEADER
