#if ! defined( RESPONSE_CLASS_HEADER )

#define RESPONSE_CLASS_HEADER

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

class NMEA0183;

class RESPONSE
{
   private:

      NMEA0183 *container_p;

   public:

      RESPONSE();
      virtual ~RESPONSE();

      /*
      ** Data
      */

      std::string DataSource;
      std::string ErrorMessage;
      std::string Mnemonic;
      std::string Talker;

      /*
      ** Methods
      */

      virtual void Empty( void ) = 0;
      virtual bool Parse( const SENTENCE& sentence ) = 0;
      virtual std::string PlainEnglish( void ) const;
      virtual void SetErrorMessage( const std::string& message );
      virtual void SetContainer( NMEA0183 *container );
      virtual bool Write( SENTENCE& sentence );
};
 
#endif // RESPONSE_CLASS_HEADER
