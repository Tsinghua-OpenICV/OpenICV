#if ! defined( XDR_CLASS_HEADER )

#define XDR_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: xdr.hpp $
** $Revision: 5 $
** $Modtime: 10/10/98 3:03p $
*/

class TRANSDUCER_DATA
{
   public:

      TRANSDUCER_DATA();
	   virtual ~TRANSDUCER_DATA();

	   /*
      ** Data
      */

	   TRANSDUCER_TYPE TransducerType;
	   double          MeasurementData;
	   QString         MeasurementUnits;
	   QString         TransducerName;

	   /*
      ** Methods
      */

      virtual void Empty( void );
	  virtual void Parse( int first_field_number, const SENTENCE& sentence );
      virtual void Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const TRANSDUCER_DATA& operator = ( const TRANSDUCER_DATA& source );
};

class XDR : public RESPONSE
{
   private:

      void delete_all_entries( void );

   public:

      XDR();
     ~XDR();

      QVector<TRANSDUCER_DATA*> Transducers;

      virtual void Empty( void );
      virtual BOOL Parse( const SENTENCE& sentence );
      virtual BOOL Write( SENTENCE& sentence );
};

#endif // XDR_CLASS_HEADER
