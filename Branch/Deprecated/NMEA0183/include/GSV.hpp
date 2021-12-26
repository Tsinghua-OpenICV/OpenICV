#if ! defined( GSV_CLASS_HEADER )

#define GSV_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: gsv.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:47p $
** 
** Modification: Gerald Dherbomez Heudiasyc UTC/CNRS UMR 6599 Copyright
** Pass the number of max satellite in GSV frame from 3 to 9
*/

class GSV : public RESPONSE
{
   public:

      GSV();
      virtual ~GSV();

      /*
      ** Data
      */

      int            NumberOfSatellites;
	  int			 message_number;
	  int			 Totalmessages;
      SATELLITE_DATA SatellitesInView[ 36 ];

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const GSV& operator = ( const GSV& source );
};

#endif // GSV_CLASS_HEADER
