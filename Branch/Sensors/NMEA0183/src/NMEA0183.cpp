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
** $Workfile: nmea0183.cpp $
** $Revision: 5 $
** $Modtime: 10/10/98 4:27p $
*/
/*
#ifdef _DEBUG
#undef THIS_FILE
static char BASED_CODE THIS_FILE[] = __FILE__;
#define new DEBUG_NEW
#endif
*/
NMEA0183::NMEA0183()
{
   //m_Initialize();

   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Aam );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Alm );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Apb );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Asd );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Bec );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Bod );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Bwc );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Bwr );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Bww );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Dbt );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Dcn );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Dpt );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Fsi );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Gda ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Gdf ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Gdp ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Gga );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Gla ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Glc );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Glf ); // Sentence Not Recommended For New Designs

   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Gll );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Glp ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Gsa );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   // ajout de GST
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Gst );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Gsv );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Gtd ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Gxa );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Gxf ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Gxp ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Hcc ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Hdg );

   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Hdm ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Hdt );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Hsc );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Ima ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Lcd );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Mhu ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Mta ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Mtw );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Mwv );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Oln );

   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Osd );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Proprietary );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Rma );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Rmb );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Rmc );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Rot );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Rpm );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Rsa );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Rsd );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Rte );

   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Sfi );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Stn );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Tep ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Trf );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Ttm );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Vbw );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Vdr );
      //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Vhw );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Vlw );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Vpw );

   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Vtg );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Vwe );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Wcv ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Wdc ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Wdr ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Wnc );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Wpl );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Xdr );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Xte );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Xtr );

   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Zda );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Zfi ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Zfo );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Zlz ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Zpi ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Zta ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Zte ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Ztg );
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Zti ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Zwp ); // Sentence Not Recommended For New Designs
   //m_ResponseTable.resize(m_ResponseTable.size()+1);
   m_ResponseTable.insert(m_ResponseTable.size(), (RESPONSE *) &Zzu ); // Sentence Not Recommended For New Designs
   
   m_SortResponseTable();
   m_SetContainerPointers();
}

NMEA0183::~NMEA0183()
{
   //m_Initialize();
}

void NMEA0183::m_Initialize( void )
{
   //ErrorMessage.Empty();
}

void NMEA0183::m_SetContainerPointers( void )
{
   int index = 0;
   int number_of_entries_in_table = m_ResponseTable.size();

   RESPONSE *this_response = (RESPONSE *) NULL;

   index = 0;

   while( index < number_of_entries_in_table )
   {
      this_response = (RESPONSE *) m_ResponseTable[ index ];

      this_response->SetContainer( this );

      index++;
   }
}

void NMEA0183::m_SortResponseTable( void )
{
   int index = 0;
   int number_of_entries_in_table = m_ResponseTable.size() - 1;

   RESPONSE *this_response = (RESPONSE *) NULL;
   RESPONSE *that_response = (RESPONSE *) NULL;

   BOOL sorted = FALSE;

   while( sorted == FALSE )
   {
      sorted = TRUE;

      index = 0;

      while( index < number_of_entries_in_table )
      {
         this_response = (RESPONSE *) m_ResponseTable[ index     ];
         that_response = (RESPONSE *) m_ResponseTable[ index + 1 ];
		     //that_response = (RESPONSE *) m_ResponseTable.at(index + 1);

         if ( this_response->Mnemonic.compare( that_response->Mnemonic ) > 0 )
         {
            //m_ResponseTable[ index     ] = that_response;
           //m_ResponseTable[ index + 1 ] = this_response;
           //m_ResponseTable.insert ( index ,that_response );
           m_ResponseTable.replace ( index ,that_response );
           //m_ResponseTable.insert ( index+1 ,this_response );
           m_ResponseTable.replace ( index+1 ,this_response );

            sorted = FALSE;
         }

         index++;
      }
   }
}

/*
** Public Interface
*/

BOOL NMEA0183::IsGood( void ) const
{
   /*
   ** NMEA 0183 sentences begin with $ and and with CR LF
   */

   if ( m_Sentence.Sentence[ 0 ] != '$' )
   {
      return( FALSE );
   }

   /*
   ** Next to last character must be a CR
   */
//   char character[1]; 
//   QString str = (QString) m_Sentence.Sentence.mid( m_Sentence.Sentence.length() - 2, 1 ); 
//   character[0] = str.toLatin1();

   if ( m_Sentence.Sentence[m_Sentence.Sentence.length() - 2].toLatin1() != CARRIAGE_RETURN )
   {
      return( FALSE );
   }

   if ( m_Sentence.Sentence[m_Sentence.Sentence.length() - 1].toLatin1() != LINE_FEED )
   {
      return( FALSE );
   }

   return( TRUE );
}

BOOL NMEA0183::Parse( void )
{
   BOOL return_value = FALSE;

   if ( IsGood() )
   {
      int index       = 0;
      int comparison  = 0;
      int drop_dead   = 0;
      int exit_loop   = 0;
      int lower_limit = 0;
      int upper_limit = 0;

      QString mnemonic;

      RESPONSE *response_p = (RESPONSE *) NULL;

      mnemonic = m_Sentence.Field( 0 );

      /*
      ** See if this is a proprietary field
      */

      if ( mnemonic[0].toLatin1() == 'P' )
      {
         mnemonic = "P";
      }
      else
      {
         mnemonic = mnemonic.right( 3 );
      }

      /*
      ** Set up our default error message
      */

      ErrorMessage = mnemonic;
      ErrorMessage += " is an unknown type of sentence";

      LastSentenceIDReceived = mnemonic;

      /*
      ** Do a Binary Search to call the appropriate function
      */

      lower_limit = 0;
      upper_limit = m_ResponseTable.size();
      index       = upper_limit / 2;
      drop_dead   = ( index < 10 ) ? 10 : index + 2;

      /*
      ** The drop_dead is here as an insurance policy that we will never get stuck in an endless loop.
      ** I have encountered situations where the inaccuracy of the division leaves the loop stuck when
      ** it can't find something.
      */

      while( exit_loop == 0 )
      {
         response_p = (RESPONSE *) m_ResponseTable[ index ];

         comparison = mnemonic.compare( response_p->Mnemonic );

         if ( comparison == 0 )
         {
            return_value = response_p->Parse( m_Sentence );

            /*
            ** Set your ErrorMessage
            */

            if ( return_value == TRUE )
            {
               /*
               ** Now that we sucessfully parsed a sentence, record stuff *about* the transaction
               */

               ErrorMessage         = "No Error";
               LastSentenceIDParsed = response_p->Mnemonic;
               TalkerID             = talker_id( m_Sentence );
               ExpandedTalkerID     = expand_talker_id( TalkerID );
               PlainText            = response_p->PlainEnglish();
            }
            else
            {
               ErrorMessage = response_p->ErrorMessage;
            }

            exit_loop = 1;
         }
         else
         {
            if ( comparison < 0 )
            {
               upper_limit = index;
            }
            else
            {
               lower_limit = index;
            }

            if ( lower_limit == upper_limit )
            {
               exit_loop    = 1;
               return_value = FALSE;
            }
            else
            {
               index = ( lower_limit + upper_limit ) / 2;
            }
         }

         drop_dead--;

         if ( exit_loop != 1 && drop_dead < 0 )
         {
            exit_loop    = 1;
            return_value = FALSE;
         }
      }
   }
   else
   {
      return_value = FALSE;
   }

   return( return_value );
}
/*
#if 0

void NMEA0183::Serialize( CArchive& archive )
{
   CObject::Serialize( archive );

   if ( archive.IsStoring() )
   {
      archive << m_Sentence;
      archive << ErrorMessage;
      archive << LastSentenceIDParsed;
      archive << LastSentenceIDReceived;
      archive << PlainText;
      archive << TalkerID;
      archive << ExpandedTalkerID;
   }
   else
   {
      //archive >> m_Sentence;
      archive >> ErrorMessage;
      archive >> LastSentenceIDParsed;
      archive >> LastSentenceIDReceived;
      archive >> PlainText;
      archive >> TalkerID;
      archive >> ExpandedTalkerID;
   }
}

#endif // 0
*/
NMEA0183& NMEA0183::operator << ( const char *source )
{
   m_Sentence = source;

   return( *this );
}

NMEA0183& NMEA0183::operator >> ( QString& destination )
{
   destination = m_Sentence;

   return( *this );
}
