#if ! defined( RMC_CLASS_HEADER )

#define RMC_CLASS_HEADER

/*
Author: Samuel R. Blackburn
Internet: wfc@pobox.com

"You can get credit for something or get it done, but not both."
Dr. Richard Garwin

The MIT License (MIT)

Copyright (c) 1996-2015 Sam Blackburn

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

class RMC : public RESPONSE
{
   public:

      RMC();
      virtual ~RMC();

      /*
      ** Data
      */

      std::string      UTCTime;
      time_t           Time;
      int msec;
      NMEA0183_BOOLEAN IsDataValid;
      LATLONG          Position;
      double           SpeedOverGroundKnots;
      double           TrackMadeGoodDegreesTrue;
      int dd,mm,yy;
      std::string      Date;
      double           MagneticVariation;
      EASTWEST         MagneticVariationDirection;

      /*
      ** Methods
      */

      virtual void Empty( void ) override;
      virtual bool Parse( const SENTENCE& sentence ) override;
      virtual std::string PlainEnglish( void ) const override;
      virtual bool Write( SENTENCE& sentence ) override;

      /*
      ** Operators
      */

      virtual const RMC& operator = ( const RMC& source );
};

#endif // RMC_CLASS_HEADER
