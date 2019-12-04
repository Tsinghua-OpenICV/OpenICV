#include "HSATIT.hpp"
#include "nmea0183.h"



HSATIT::HSATIT()
{
   Mnemonic = "HSATIT";
   Empty();
}

HSATIT::~HSATIT()
{
   //Mnemonic.Empty();
   Empty();
}

void HSATIT::Empty( void )
{


    heading				= 0.0;
	roll				= 0.0;
	pitch				= 0.0;
	heave				= 0.0;
	yawRotationRate		= 0.0;
	rollRotationRate	= 0.0;
	pitchRotationRate	= 0.0;
	courseMadeGood		= 0.0;
	speedOverGround		= 0.0;
	longitudinalVel		= 0.0;
	transveralVel		= 0.0;
	verticalVel			= 0.0;
	headingStdDev		= 0.0;
	rollStdDev			= 0.0;
	pitchStdDev			= 0.0;
	northSpeedStdDev	= 0.0;
	eastSpeeedStdDev	= 0.0;
	verticalStdDev		= 0.0;
}

BOOL HSATIT::Parse( const SENTENCE& sentence )
{
   /*
   ** HSATIT
   **
   **               1     2     3     4   5     6     7     8     9    10    11    12    13   14   15   16   17   18
   **               |     |     |     |   |     |     |     |     |     |     |     |     |    |    |    |    |    |
   ** $HSATIT,h.hhh,r.rrr,p.ppp,h.h,a.aaa,b.bbb,c.ccc,d.ddd,e.eee,f.fff,g.ggg,h.hhh,i.ii,j.jj,k.kk,l.ll,m.mm,n.nn<CR><LF>
   **
   ** Field Number:
   **   1)  h.hhh		is the heading in deg
		2)  r.rrr		is the roll in deg
		3)  p.ppp		is the pitch in deg
		4)  h.h			is the heave in meters
		5)  a.aaa*		is the X3 rotation rate* in deg/s
		6)  b.bbb*		is the X1 rotation rate* in deg/s
		7)  c.ccc*		is the X2 rotation rate* in deg/s
		8) d.ddd		is the course made good in deg
		9) e.eee		is the speed over ground
		10) d.dd		is the longitudinal velocity in m/s (positive towards the bow)
		11) g.ggg		is the transverse velocity in m/s (positive towards port side)
		12) h.hhh		is the vertical velocity in m/s (positive towards up side)
		13) i.ii		is the heading Std in deg
		14) j.jj		is the roll Std in deg
		15) k.kk		is the pitch Std in deg
		16) l.ll		is the north speed Std in m/s
		17) m.mm		is the east speed Std in m/s
		18) n.nn		is the vertical speed Std in m/s


   */

    heading				= sentence.Double( 1 );
	roll				= sentence.Double( 2 );
	pitch				= sentence.Double( 3 );
	heave				= sentence.Double( 4 );
	yawRotationRate		= sentence.Double( 5 );
	rollRotationRate	= sentence.Double( 6 );
	pitchRotationRate	= sentence.Double( 7 );
	courseMadeGood		= sentence.Double( 8 );
	speedOverGround		= sentence.Double( 9 );
	longitudinalVel		= sentence.Double( 10 );
	transveralVel		= sentence.Double( 11 );
	verticalVel			= sentence.Double( 12 );
	headingStdDev		= sentence.Double( 13 );
	rollStdDev			= sentence.Double( 14 );
	pitchStdDev			= sentence.Double( 15 );
	northSpeedStdDev	= sentence.Double( 16 );
	eastSpeeedStdDev	= sentence.Double( 17 );
	verticalStdDev		= sentence.Double( 18 );

   return( TRUE );
}

QString HSATIT::PlainEnglish( void ) const
{
   QString return_string;

   return_string = "not yet available ";

   return( return_string );
}

BOOL HSATIT::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */

   RESPONSE::Write( sentence );

	sentence += heading;
	sentence += roll;
	sentence += pitch;
	sentence += sentence += sentence += heave;
	sentence += yawRotationRate;
	sentence += rollRotationRate;
	sentence += pitchRotationRate;
	sentence += courseMadeGood;
	sentence += speedOverGround;
	sentence += longitudinalVel;
	sentence += transveralVel;
	sentence += verticalVel;
	sentence += headingStdDev;
	sentence += rollStdDev;
	sentence += pitchStdDev;
	sentence += sentence += northSpeedStdDev;
	sentence += eastSpeeedStdDev;
	sentence += verticalStdDev;

   sentence.Finish();

   return( TRUE );
}

const HSATIT& HSATIT::operator = ( const HSATIT& source )
{

    heading				= source.heading;
	roll				= source.roll;
	pitch				= source.pitch;
	heave				= source.heave;
	yawRotationRate		= source.yawRotationRate;
	rollRotationRate	= source.rollRotationRate;
	pitchRotationRate	= source.pitchRotationRate;
	courseMadeGood		= source.courseMadeGood;
	speedOverGround		= source.speedOverGround;
	longitudinalVel		= source.longitudinalVel;
	transveralVel		= source.transveralVel;
	verticalVel			= source.verticalVel;
	headingStdDev		= source.headingStdDev;
	rollStdDev			= source.rollStdDev;
	pitchStdDev			= source.pitchStdDev;
	northSpeedStdDev	= source.northSpeedStdDev;
	eastSpeeedStdDev	= source.eastSpeeedStdDev;
	verticalStdDev		= source.verticalStdDev;

   return( *this );
}
