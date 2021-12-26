#if ! defined( LATLONG_CLASS_HEADER )

#define LATLONG_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: latlong.hpp $
** $Revision: 6 $
** $Modtime: 10/13/98 6:23a $
*/

class SENTENCE;

class COORDINATE
{
   public:

      COORDINATE() { Coordinate = 0.0; }
     ~COORDINATE() { Coordinate = 0.0; }

      double Coordinate;

      double GetDecimalDegrees( void ) const;
      double GetDecimalMinutes( void ) const;
      double GetDecimalSeconds( void ) const;
      int    GetWholeDegrees( void ) const;
      int    GetWholeMinutes( void ) const;
      int    GetWholeSeconds( void ) const;
};

class LATITUDE : public COORDINATE
{
   public:

      LATITUDE();
      virtual ~LATITUDE();

      /*
      ** Data
      */

      double Latitude;

      NORTHSOUTH Northing;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual bool IsDataValid( void );
      virtual void Parse( int PositionFieldNumber, int NorthingFieldNumber, const SENTENCE& LineToParse );
      virtual void Set( double Position, const char *Northing );
      virtual void Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const LATITUDE& operator = ( const LATITUDE& source );
};

class LONGITUDE : public COORDINATE
{
   public:

      LONGITUDE();
      virtual ~LONGITUDE();

      /*
      ** Data
      */

      double Longitude;

      EASTWEST Easting;

      /*
      ** Methods
      */

      virtual void Empty( void );
      virtual bool IsDataValid( void );
      virtual void Parse( int PositionFieldNumber, int EastingFieldNumber, const SENTENCE& LineToParse );
      virtual void Set( double Position, const char *Easting );
      virtual void Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const LONGITUDE& operator = ( const LONGITUDE& source );
};

class LATLONG
{
   public:

      LATLONG();
      virtual ~LATLONG();

      /*
      ** Data
      */

      LATITUDE  Latitude;
      LONGITUDE Longitude;

      /*
      ** Methods
      */
       
      virtual void Empty( void );
      virtual bool Parse( int LatitudePostionFieldNumber, int NorthingFieldNumber, int LongitudePositionFieldNumber, int EastingFieldNumber, const SENTENCE& LineToParse );
      virtual QString PlainEnglish( void ) const;
      virtual void Write( SENTENCE& sentence );

      /*
      ** Operators
      */

      virtual const LATLONG& operator = ( const LATLONG& source );
};

#endif // LATLONG_CLASS_HEADER
