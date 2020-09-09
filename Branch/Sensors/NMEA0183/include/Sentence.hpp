#if ! defined( SENTENCE_CLASS_HEADER )

#define SENTENCE_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: sentence.hpp $
** $Revision: 5 $
** $Modtime: 10/10/98 10:04a $
*/

class LATLONG;

class SENTENCE
{
   public:

      SENTENCE();
      virtual ~SENTENCE();

      /*
      ** Data
      */

      QString Sentence;

      /*
      ** Methods
      */

      virtual NMEA0183_BOOLEAN Boolean( int field_number ) const;
      virtual unsigned char ComputeChecksum( void ) const;
      virtual COMMUNICATIONS_MODE CommunicationsMode( int field_number ) const;
      virtual double Double( int field_number ) const;
      virtual EASTWEST EastOrWest( int field_number ) const;
      virtual const QString Field( int field_number ) const;
      virtual void Finish( void );
      virtual unsigned short GetNumberOfDataFields( void ) const;
      virtual int Integer( int field_number ) const;
      virtual NMEA0183_BOOLEAN IsChecksumBad( int checksum_field_number ) const;
      virtual LEFTRIGHT LeftOrRight( int field_number ) const;
      virtual NORTHSOUTH NorthOrSouth( int field_number ) const;
      virtual REFERENCE Reference( int field_number ) const;
      virtual const QDateTime Time( int field_number ) const;
      virtual TRANSDUCER_TYPE TransducerType( int field_number ) const;
      virtual const QString getAutomaticChecksum( void ) const; 

      /*
      ** Operators
      */

      operator QString() const; 
      virtual const SENTENCE& operator  = ( const SENTENCE& source );
      virtual const SENTENCE& operator  = ( const QString& source );
      virtual const SENTENCE& operator += ( const QString& source );
      virtual const SENTENCE& operator += ( double value );
      virtual const SENTENCE& operator += ( int value );
      virtual const SENTENCE& operator += ( COMMUNICATIONS_MODE mode );
      virtual const SENTENCE& operator += ( EASTWEST easting );
      virtual const SENTENCE& operator += ( LATLONG& source );
      virtual const SENTENCE& operator += ( NMEA0183_BOOLEAN boolean );
      virtual const SENTENCE& operator += ( NORTHSOUTH northing );
      virtual const SENTENCE& operator += ( const QDateTime time );
      virtual const SENTENCE& operator += ( TRANSDUCER_TYPE transducer );
};
 
#endif // SENTENCE_CLASS_HEADER
