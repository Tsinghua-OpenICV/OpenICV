#include "nmea0183.h"

const QString& field( int desired_field_number, const QString& sentence )
{
   static QString return_string;

   //return_string.Empty();

   int index                = 1; // Skip over the $ at the begining of the sentence
   int return_index         = 0;
   int current_field_number = 0;
   int string_length        = 0;

   string_length = sentence.length();

   while( current_field_number < desired_field_number && index < string_length )
   {
      if ( sentence[ index ] == ',' || sentence[ index ] == '*' )
      {
         current_field_number++;
      }
      else
      {
         /*
         ** Do Nothing
         */
      }

      index++;
   }

   if ( current_field_number == desired_field_number )
   {
      while( index < string_length    &&
             sentence[ index ] != ',' &&
             sentence[ index ] != '*' &&
             sentence[ index ] != (char)0x00 )
      {
         return_string += sentence[ index ];
         index++;
      }
   }
   else
   {
      /*
      ** Do Nothing
      */
   }

   return( return_string );
}
