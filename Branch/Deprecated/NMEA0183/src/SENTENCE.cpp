#include "nmea0183.h"
#pragma hdrstop

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: sentence.cpp $
** $Revision: 5 $
** $Modtime: 10/10/98 2:43p $
*/
/*
#ifdef _DEBUG
#undef THIS_FILE
static char BASED_CODE THIS_FILE[] = __FILE__;
#endif
*/

#ifdef _MSC_VER
#   pragma warning(disable:4996)
#endif // _MSC_VER

#ifndef WIN32
typedef unsigned char BYTE;
#endif

SENTENCE::SENTENCE()
{
   //Sentence.Empty();
}

SENTENCE::~SENTENCE()
{
   //Sentence.Empty();
}

NMEA0183_BOOLEAN SENTENCE::Boolean( int field_number ) const
{
   QString field_data;

   field_data = Field( field_number );

   if ( field_data == "A" )
   {
      return( True );
   }
   else if ( field_data == "V" )
   {
      return( False );
   }
   else
   {
      return( NMEA_Unknown );
   }
}

COMMUNICATIONS_MODE SENTENCE::CommunicationsMode( int field_number ) const
{
   QString field_data;

   field_data = Field( field_number );

   if ( field_data == "d" )
   {		 
      return( F3E_G3E_SimplexTelephone );
   }
   else if ( field_data == "e" )
   {
      return( F3E_G3E_DuplexTelephone );
   }
   else if ( field_data == "m" )
   {
      return( J3E_Telephone );
   }
   else if ( field_data == "o" )
   {
      return( H3E_Telephone );
   }
   else if ( field_data == "q" )
   {
      return( F1B_J2B_FEC_NBDP_TelexTeleprinter );
   }
   else if ( field_data == "s" )
   {
      return( F1B_J2B_ARQ_NBDP_TelexTeleprinter );
   }
   else if ( field_data == "w" )
   {
      return( F1B_J2B_ReceiveOnlyTeleprinterDSC );
   }
   else if ( field_data == "x" )
   {
      return( A1A_MorseTapeRecorder );
   }
   else if ( field_data == "{" )
   {
      return( A1A_MorseKeyHeadset );
   }
   else if ( field_data == "|" )
   {
      return( F1C_F2C_F3C_FaxMachine );
   }
   else
   {
      return( CommunicationsModeUnknown );
   }
}

unsigned char SENTENCE::ComputeChecksum( void ) const
{
   unsigned char checksum_value = 0;

   int string_length = Sentence.length();
   int index = 1; // Skip over the $ at the begining of the sentence

   while( index < string_length    && 
          Sentence[ index ] != '*' && 
          Sentence[ index ] != (char)CARRIAGE_RETURN && 
          Sentence[ index ] != (char)LINE_FEED )
   {
      checksum_value ^= Sentence[ index ].toLatin1();
      index++;
   }

   return( checksum_value );
}

double SENTENCE::Double( int field_number ) const
{
   QString field_data = "";

   field_data = Field( field_number );

   return(  field_data.toDouble() );
}

EASTWEST SENTENCE::EastOrWest( int field_number ) const
{
   QString field_data;

   field_data = Field( field_number );

   if ( field_data == "E" )
   {
      return( East );
   }
   else if ( field_data == "W" )
   {
      return( West );
   }
   else
   {
      return( EW_Unknown );
   }
}

const QString SENTENCE::Field( int desired_field_number ) const
{
   // Thanks to Vilhelm Persson (vilhelm.persson@st.se) for finding a 
   // bug that lived here.

   QString return_string = "";

   //return_string.Empty();

   int index                = 1; // Skip over the $ at the begining of the sentence
   int current_field_number = 0;
   int string_length        = 0;

   string_length = Sentence.length();

   while( current_field_number < desired_field_number && index < string_length )
   {
      if ( Sentence[ index ] == ',' || Sentence[ index ] == '*' )
      {
         current_field_number++;
      }

      index++;
   }

   if ( current_field_number == desired_field_number )
   {
      while( index < string_length    &&
             Sentence[ index ] != ',' &&
             Sentence[ index ] != '*' &&
             Sentence[ index ] != (char)0x00 )
      {
         return_string += Sentence[ index ];
         index++;
      }
   }

   return( return_string );
}

WORD SENTENCE::GetNumberOfDataFields( void ) const
{
   int index                = 1; // Skip over the $ at the begining of the sentence
   int current_field_number = 0;
   int string_length        = 0;

   string_length = Sentence.length();

   while( index < string_length )
   {
      if ( Sentence[ index ] == '*' )
      {
         return( (WORD) current_field_number );
      }

      if ( Sentence[ index ] == ',' )
      {
         current_field_number++;
      }

      index++;
   }

   return( (WORD) current_field_number );
}

void SENTENCE::Finish( void )
{
   BYTE checksum = ComputeChecksum();

   char temp_string[ 10 ];

   ::sprintf( temp_string, "*%02X%c%c", (int) checksum, CARRIAGE_RETURN, LINE_FEED );

   Sentence += temp_string;
}

int SENTENCE::Integer( int field_number ) const
{
   QString integer_string = "";

   integer_string = Field( field_number );

   return( ::atoi( integer_string.toLatin1() ) );
}

NMEA0183_BOOLEAN SENTENCE::IsChecksumBad( int checksum_field_number ) const
{
/*
** Checksums are optional, return TRUE if an existing checksum is known to be bad
*/
  
  QString checksum_in_sentence; 
  
  if (checksum_field_number == 0)
    checksum_in_sentence = getAutomaticChecksum(); 
  else 
    checksum_in_sentence = Field( checksum_field_number );
  
  if ( checksum_in_sentence == "" )
  {
    return( NMEA_Unknown );
  }
  
  if ( ComputeChecksum() != HexValue( checksum_in_sentence.toLatin1() ) )
  {
    return( True );
  } 
  
  return( False );
}

/*! Get automatically the checksum in the sentence : 
*   the 2 last characters after '*'
*/    
const QString SENTENCE::getAutomaticChecksum( void ) const
{
  int positionOfAsterisk = Sentence.indexOf('*',1);
  if (positionOfAsterisk != -1)
    return Sentence.mid(positionOfAsterisk+1, 2);   
  else 
    return ""; 
}

LEFTRIGHT SENTENCE::LeftOrRight( int field_number ) const
{
   QString field_data;

   field_data = Field( field_number );

   if ( field_data == "L" )
   {
      return( Left );
   }
   else if ( field_data == "R" )
   {
      return( Right );
   }
   else
   {
      return( LR_Unknown );
   }
}

NORTHSOUTH SENTENCE::NorthOrSouth( int field_number ) const
{
   QString field_data;

   field_data = Field( field_number );

   if ( field_data == "N" )
   {
      return( North );
   }
   else if ( field_data == "S" )
   {
      return( South );
   }
   else
   {
      return( NS_Unknown );
   }
}

REFERENCE SENTENCE::Reference( int field_number ) const
{
   QString field_data;

   field_data = Field( field_number );

   if ( field_data == "B" )
   {
      return( BottomTrackingLog );
   }
   else if ( field_data == "M" )
   {
      return( ManuallyEntered );
   }
   else if ( field_data == "W" )
   {
      return( WaterReferenced );
   }
   else if ( field_data == "R" )
   {
      return( RadarTrackingOfFixedTarget );
   }
   else if ( field_data == "P" )
   {
      return( PositioningSystemGroundReference );
   }
   else
   {
      return( ReferenceUnknown );
   }
}

const QDateTime SENTENCE::Time( int field_number ) const
{
   QDateTime return_value;

   return_value = return_value.currentDateTime();

   QString temp_string = Field( field_number );

   if ( temp_string.length() >= 6 )
   {
      char temp_number[ 3 ];

      temp_number[ 2 ] = 0x00;

      temp_number[ 0 ] = temp_string[ 0 ].toLatin1();
      temp_number[ 1 ] = temp_string[ 1 ].toLatin1();

      int hours = ::atoi( temp_number );

      temp_number[ 0 ] = temp_string[ 2 ].toLatin1();
      temp_number[ 1 ] = temp_string[ 3 ].toLatin1();

      int minutes = ::atoi( temp_number );

      temp_number[ 0 ] = temp_string[ 4 ].toLatin1();
      temp_number[ 1 ] = temp_string[ 5 ].toLatin1();

      int seconds = ::atoi( temp_number );

	  temp_number[ 0 ] = temp_string[ 7 ].toLatin1();
      temp_number[ 1 ] = temp_string[ 8 ].toLatin1();

      int milliseconds = ::atoi( temp_number )*10;

      int year  = return_value.date().year();
      int month = return_value.date().month();
      int day   = return_value.date().day();

      //return_value = CTime( year, month, day, hours, minutes, seconds );
	  return_value = QDateTime(QDate(year, month, day) , QTime(hours, minutes, seconds, milliseconds) ); 
   }

   return( return_value );
}

TRANSDUCER_TYPE SENTENCE::TransducerType( int field_number ) const
{
   QString field_data;

   field_data = Field( field_number );

   if ( field_data == "A" )
   {		 
      return( AngularDisplacementTransducer );
   }
   else if ( field_data == "D" )
   {
      return( LinearDisplacementTransducer );
   }
   else if ( field_data == "C" )
   {
      return( TemperatureTransducer );
   }
   else if ( field_data == "F" )
   {
      return( FrequencyTransducer );
   }
   else if ( field_data == "N" )
   {
      return( ForceTransducer );
   }
   else if ( field_data == "P" )
   {
      return( PressureTransducer );
   }
   else if ( field_data == "R" )
   {
      return( FlowRateTransducer );
   }
   else if ( field_data == "T" )
   {
      return( TachometerTransducer );
   }
   else if ( field_data == "H" )
   {
      return( HumidityTransducer );
   }
   else if ( field_data == "V" )
   {
      return( VolumeTransducer );
   }
   else
   {
      return( TransducerUnknown );
   }
}

/*
** Operators
*/

SENTENCE::operator QString() const
{
   return( Sentence );
}

const SENTENCE& SENTENCE::operator = ( const SENTENCE& source )
{
   Sentence = source.Sentence;

   return( *this );
}

const SENTENCE& SENTENCE::operator = ( const QString& source )
{
   Sentence = source;

   return( *this );
}

const SENTENCE& SENTENCE::operator += ( const QString& source )
{
   Sentence += ",";
   Sentence += source;

   return( *this );
}

const SENTENCE& SENTENCE::operator += ( double value )
{
   char temp_string[ 80 ];

   ::sprintf( temp_string, "%.3f", value );

   Sentence += ",";
   Sentence += temp_string;

   return( *this );
}

const SENTENCE& SENTENCE::operator += ( COMMUNICATIONS_MODE mode )
{
   Sentence += ",";

   switch( mode )
   {
      case F3E_G3E_SimplexTelephone:

	      Sentence += "d";
		   break;

      case F3E_G3E_DuplexTelephone:

	      Sentence += "e";
		   break;

      case J3E_Telephone:

	      Sentence += "m";
		   break;

      case H3E_Telephone:

	      Sentence += "o";
		   break;

      case F1B_J2B_FEC_NBDP_TelexTeleprinter:

	      Sentence += "q";
		   break;

      case F1B_J2B_ARQ_NBDP_TelexTeleprinter:

	      Sentence += "s";
		   break;

      case F1B_J2B_ReceiveOnlyTeleprinterDSC:

	      Sentence += "w";
		   break;

      case A1A_MorseTapeRecorder:

	      Sentence += "x";
		   break;

      case A1A_MorseKeyHeadset:

	      Sentence += "{";
		   break;

      case F1C_F2C_F3C_FaxMachine:

	      Sentence += "|";
		   break;
   }

   return( *this );
}

const SENTENCE& SENTENCE::operator += ( TRANSDUCER_TYPE transducer )
{
   Sentence += ",";

   switch( transducer )
   {
      case TemperatureTransducer:

	      Sentence += "C";
		   break;

      case AngularDisplacementTransducer:

	      Sentence += "A";
		   break;

      case LinearDisplacementTransducer:

	      Sentence += "D";
		   break;

      case FrequencyTransducer:

	      Sentence += "F";
		   break;

      case ForceTransducer:

	      Sentence += "N";
		   break;

      case PressureTransducer:

	      Sentence += "P";
		   break;

      case FlowRateTransducer:

	      Sentence += "R";
		   break;

      case TachometerTransducer:

	      Sentence += "T";
		   break;

      case HumidityTransducer:

	      Sentence += "H";
		   break;

      case VolumeTransducer:

	      Sentence += "V";
		   break;
   }

   return( *this );
}

const SENTENCE& SENTENCE::operator += ( NORTHSOUTH northing )
{
   Sentence += ",";

   if ( northing == North )
   {
      Sentence += "N";
   }
   else if ( northing == South )
   {
      Sentence += "S";
   }

   return( *this );
}

const SENTENCE& SENTENCE::operator += ( int value )
{
   char temp_string[ 80 ];

   ::sprintf( temp_string, "%d", value );

   Sentence += ",";
   Sentence += temp_string;

   return( *this );
}

const SENTENCE& SENTENCE::operator += ( EASTWEST easting )
{
   Sentence += ",";

   if ( easting == East )
   {
      Sentence += "E";
   }
   else if ( easting == West )
   {
      Sentence += "W";
   }

   return( *this );
}

const SENTENCE& SENTENCE::operator += ( NMEA0183_BOOLEAN boolean )
{
   Sentence += ",";

   if ( boolean == True )
   {
      Sentence += "A";
   }
   else if ( boolean == False )
   {
      Sentence += "V";
   }

   return( *this );
}

const SENTENCE& SENTENCE::operator += ( LATLONG& source )
{
   source.Write( *this );

   return( *this );
}

const SENTENCE& SENTENCE::operator += ( const QDateTime time )
{
   QString temp_string = time.time().toString();

   Sentence += ",";
   Sentence += temp_string;

   return( *this );
}
