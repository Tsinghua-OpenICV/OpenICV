#include "HSPOS.hpp"
//#pragma hdrstop

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: HSPOS.cpp $
** $Revision: 6 $
** $Modtime: 10/12/98 6:39a $
*/



HSPOS::HSPOS()
{
   Mnemonic = "HSPOS";
   Empty();
}

HSPOS::~HSPOS()
{
   //Mnemonic.Empty();
   Empty();
}

void HSPOS::Empty( void )
{

	
   //UTCTime.Empty();
    Position.Empty();
    depth                      = 0.0;
	altitude                   = 0.0;
	latitudeStdDev             = 0.0;
	longitudeStdDev            = 0.0; 
	latLongCov                 = 0.0; 
	depthStdDev				   = 0.0;
	UTMzone                    = 0; 
	UTMzoneChar                = ' ';
	eastindProjection          = 0.0; 
	northingProjection         = 0.0; 
	logMislignment             = 0.0; 
	logScaleFactorError        = 0.0; 
	compensationSoundVelocity  = 0.0; 
}

BOOL HSPOS::Parse( const SENTENCE& sentence )
{
   /*
   ** HSPOS - Global Positioning System Fix Data
   ** Time, Position and fix related data fora GPS receiver.
   **
   **         1         2          3 4          5 6    7    8    9   10   11   12 1314  15  16     17     18
   **         |         |          | |          | |    |    |    |    |    |    |  | |   |   |      |      |
   ** $HSPOS_,hhmmss.ss,llmm.mmmmm,H,LLmm.mmmmm,D,d.dd,a.ad,x.xx,y.yy,z.zz,d.dd,nn,c,e.e,n.n,m.mmmm,s.ssss,vvvv.v<CR><LF>
   **
   ** Field Number: 
   **   1)  hhmmss.ss	is the UTC absolute time
		2)  llmm.mmmmm  is the latitude in deg, decimal in min
		3)  H			N: north, S: south
		4)  LLmm.mmmmm is the longitude in deg, decimal in min
		5)  D			E: east, W: west
		6)  d.dd		is the depth in meters
		7)  a.aa		is the altitude in meters (from DVL)
		8)  x.xx		is the latitude Std (meters)
		9)  y.yy		is the longitude Std (meters)
		10) z.zz		is the latitude longitude covariance (meters)
		11) d.dd		is the depth Std (meters)
		12) nn			is the UTM zone integer
		13) c			is the UTM zone character
		14) e.e			is the easting projection
		15) n.n			is the northing projection
		16) m.mmmm		is the log misalignment estimation in degrees
		17) s.ssss		is the log scale factor error estimation in %
		18) vvvv.v		is the compensation sound velocity in m/s
		
		
   */

    UTCTime                    = sentence.Field( 1 );
	Time					   = sentence.Time( 1 );
    Position.Parse( 2, 3, 4, 5, sentence );
    depth                      = sentence.Double( 6 );
	altitude                   = sentence.Double( 7 );
	latitudeStdDev             = sentence.Double( 8 );
	longitudeStdDev            = sentence.Double( 9 ); 
	latLongCov                 = sentence.Double( 10 ); 
	depthStdDev				   = sentence.Double( 11 );
	UTMzone                    = sentence.Integer( 12 ); 
	UTMzoneChar                = sentence.Field(13).at(0);
	eastindProjection          = sentence.Double( 14 );
	northingProjection         = sentence.Double( 15 ); 
	logMislignment             = sentence.Double( 16 ); 
	logScaleFactorError        = sentence.Double( 17 );
	compensationSoundVelocity  = sentence.Double( 18 ); 

   return( TRUE );
}

QString HSPOS::PlainEnglish( void ) const
{
   QString return_string;

   return_string = "not yet available ";

   return( return_string );
}

BOOL HSPOS::Write( SENTENCE& sentence )
{
   /*
   ** Let the parent do its thing
   */
   
   RESPONSE::Write( sentence );

   sentence += UTCTime;
   sentence += Position;
   sentence += depth;
   sentence += altitude;
   sentence += latitudeStdDev;
   sentence += longitudeStdDev;
   sentence += latLongCov;
   sentence += depthStdDev;
   sentence += UTMzone;
   sentence += UTMzoneChar;
   sentence += eastindProjection;
   sentence += northingProjection;
   sentence += logMislignment;
   sentence += logScaleFactorError;
   sentence += compensationSoundVelocity;

   sentence.Finish();

   return( TRUE );
}

const HSPOS& HSPOS::operator = ( const HSPOS& source )
{
    UTCTime                    =  source.UTCTime;
    Position				   =  source.Position;
    depth                      =  source.depth;
	altitude                   =  source.altitude;
	latitudeStdDev             =  source.latitudeStdDev;
	longitudeStdDev            =  source.longitudeStdDev; 
	latLongCov                 =  source.latLongCov; 
	depthStdDev				   =  source.depthStdDev;
	UTMzone                    =  source.UTMzone; 
	UTMzoneChar                =  source.UTMzoneChar;
	eastindProjection          =  source.eastindProjection;
	northingProjection         =  source.northingProjection; 
	logMislignment             =  source.logMislignment;
	logScaleFactorError        =  source.logScaleFactorError;
	compensationSoundVelocity  =  source.compensationSoundVelocity; 

   return( *this );
}
