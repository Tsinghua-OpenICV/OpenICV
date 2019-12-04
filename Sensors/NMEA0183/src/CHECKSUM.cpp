#include "nmea0183.h"

unsigned char ComputeChecksum( const QString &line_to_parse )
{
   unsigned char checksum_value = 0;

   int string_length = line_to_parse.length();
   int index = 1; // Skip over the $ at the begining of the sentence

   while( index < string_length && line_to_parse[ index ] != '*' )
   {
      checksum_value ^= line_to_parse[ index ].toLatin1();
      index++;
   }

   return( checksum_value );
}
