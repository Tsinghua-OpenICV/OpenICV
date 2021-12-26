#if ! defined( NMEA_0183_CLASS_HEADER )

#define NMEA_0183_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: nmea0183.hpp $
** $Revision: 5 $
** $Modtime: 10/10/98 3:03p $
*/
#ifndef WIN32
typedef unsigned short WORD;
#endif

/*
** General Purpose Classes
*/

#include "Sentence.hpp"
#include "Response.hpp"
#include "LatLong.hpp"
#include "LoranTD.hpp"
#include "Manufact.hpp"
#include "MList.hpp"
#include "OmegaPar.hpp"
#include "DeccaLOP.hpp"
#include "RatioPls.hpp"
#include "RadarDat.hpp"
#include "SatDat.hpp"
#include "FreqMode.hpp"
#include "WayptLoc.hpp" // Sentence Not Recommended For New Designs

/*
** Response Classes
*/

#include "AAM.hpp"
#include "ALM.hpp"
#include "APB.hpp"
#include "ASD.hpp"
#include "BEC.hpp"
#include "BOD.hpp"
#include "BWC.hpp"
#include "BWR.hpp"
#include "BWW.hpp"
#include "DBT.hpp"
#include "DCN.hpp"
#include "DPT.hpp"
#include "FSI.hpp"
#include "GDA.hpp" // Sentence Not Recommended For New Designs
#include "GDF.hpp" // Sentence Not Recommended For New Designs
#include "GDP.hpp" // Sentence Not Recommended For New Designs
#include "GGA.hpp"
#include "GLA.hpp" // Sentence Not Recommended For New Designs
#include "GLC.hpp"
#include "GLF.hpp" // Sentence Not Recommended For New Designs
#include "GLL.hpp"
#include "GLP.hpp" // Sentence Not Recommended For New Designs
#include "GOA.hpp" // Sentence Not Recommended For New Designs
#include "GOF.hpp" // Sentence Not Recommended For New Designs
#include "GOP.hpp" // Sentence Not Recommended For New Designs
#include "GSA.hpp"
#include "GST.hpp" // ajout?par moi
#include "GSV.hpp"
#include "GTD.hpp" // Sentence Not Recommended For New Designs
#include "GXA.hpp"
#include "GXF.hpp" // Sentence Not Recommended For New Designs
#include "GXP.hpp" // Sentence Not Recommended For New Designs
#include "HCC.hpp" // Sentence Not Recommended For New Designs
#include "HDG.hpp"
#include "HDM.hpp" // Sentence Not Recommended For New Designs
#include "HDT.hpp"
#include "HSC.hpp"
#include "IMA.hpp" // Sentence Not Recommended For New Designs
#include "LCD.hpp"
#include "MHU.hpp" // Sentence Not Recommended For New Designs
#include "MTA.hpp" // Sentence Not Recommended For New Designs
#include "MTW.hpp"
#include "MWV.hpp"
#include "OLN.hpp"
#include "OSD.hpp"
#include "P.hpp"
#include "RMA.hpp"
#include "RMB.hpp"
#include "RMC.hpp"
#include "ROT.hpp"
#include "RPM.hpp"
#include "RSA.hpp"
#include "RSD.hpp"
#include "RTE.hpp"
#include "SFI.hpp"
#include "STN.hpp"
#include "TEP.hpp" // Sentence Not Recommended For New Designs
#include "TRF.hpp"
#include "TTM.hpp"
#include "VBW.hpp"
#include "VDR.hpp"
#include "VHW.hpp"
#include "VLW.hpp"
#include "VPW.hpp"
#include "VTG.hpp"
#include "VWE.hpp" // Sentence Not Recommended For New Designs
#include "WCV.hpp"
#include "WDC.hpp" // Sentence Not Recommended For New Designs
#include "WDR.hpp" // Sentence Not Recommended For New Designs
#include "WNC.hpp"
#include "WPL.hpp"
#include "XDR.hpp"
#include "XTE.hpp"
#include "XTR.hpp"
#include "ZDA.hpp"
#include "ZFI.hpp" // Sentence Not Recommended For New Designs
#include "ZFO.hpp" 
#include "ZLZ.hpp" // Sentence Not Recommended For New Designs
#include "ZPI.hpp" // Sentence Not Recommended For New Designs
#include "ZTA.hpp" // Sentence Not Recommended For New Designs
#include "ZTE.hpp" // Sentence Not Recommended For New Designs
#include "ZTG.hpp"
#include "ZTI.hpp" // Sentence Not Recommended For New Designs
#include "ZWP.hpp" // Sentence Not Recommended For New Designs
#include "ZZU.hpp" // Sentence Not Recommended For New Designs


class NMEA0183
{
   private:

      SENTENCE m_Sentence;

      void m_Initialize( void );

   protected:

      QVector<RESPONSE*> m_ResponseTable;

      void m_SetContainerPointers( void );
      void m_SortResponseTable( void );

   public:

      NMEA0183();
      virtual ~NMEA0183();

      /*
      ** NMEA 0183 Sentences we understand
      */

      AAM Aam;
      ALM Alm;
      APB Apb;
      ASD Asd;
      BEC Bec;
      BOD Bod;
      BWC Bwc;
      BWR Bwr;
      BWW Bww;
      DBT Dbt;
      DCN Dcn;
      DPT Dpt;
      FSI Fsi;
      GDA Gda; // Sentence Not Recommended For New Designs
      GDF Gdf; // Sentence Not Recommended For New Designs
      GDP Gdp; // Sentence Not Recommended For New Designs
      GGA Gga;
      GLA Gla; // Sentence Not Recommended For New Designs
      GLC Glc;
      GLF Glf; // Sentence Not Recommended For New Designs
      GLL Gll;
      GLP Glp; // Sentence Not Recommended For New Designs
      GOA Goa; // Sentence Not Recommended For New Designs
      GOF Gof; // Sentence Not Recommended For New Designs
      GOP Gop; // Sentence Not Recommended For New Designs
      GSA Gsa;
	  GST Gst; // ajout?par moi
      GSV Gsv;
      GTD Gtd; // Sentence Not Recommended For New Designs
      GXA Gxa;
      GXF Gxf; // Sentence Not Recommended For New Designs
      GXP Gxp; // Sentence Not Recommended For New Designs
      HCC Hcc; // Sentence Not Recommended For New Designs
      HDG Hdg;
      HDM Hdm; // Sentence Not Recommended For New Designs
      HDT Hdt;
      HSC Hsc;
      IMA Ima; // Sentence Not Recommended For New Designs
      LCD Lcd;
      MHU Mhu; // Sentence Not Recommended For New Designs
      MTA Mta; // Sentence Not Recommended For New Designs
      MTW Mtw;
      MWV Mwv;
      OLN Oln;
      OSD Osd;
      P   Proprietary;
      RMA Rma;
      RMB Rmb;
      RMC Rmc;
      ROT Rot;
      RPM Rpm;
      RSA Rsa;
      RSD Rsd;
      RTE Rte;
      SFI Sfi;
	  STN Stn;
      TEP Tep; // Sentence Not Recommended For New Designs
      TRF Trf;
      TTM Ttm;
      VBW Vbw;
      VDR Vdr;
      VHW Vhw;
      VLW Vlw;
      VPW Vpw;
      VTG Vtg;
      VWE Vwe; // Sentence Not Recommended For New Designs
      WCV Wcv;
      WDC Wdc; // Sentence Not Recommended For New Designs
      WDR Wdr; // Sentence Not Recommended For New Designs
      WNC Wnc;
      WPL Wpl;
	  XDR Xdr;
	  XTE Xte;
      XTR Xtr;
      ZDA Zda;
      ZFI Zfi; // Sentence Not Recommended For New Designs
      ZFO Zfo;
      ZLZ Zlz; // Sentence Not Recommended For New Designs
      ZPI Zpi; // Sentence Not Recommended For New Designs
      ZTA Zta; // Sentence Not Recommended For New Designs
      ZTE Zte; // Sentence Not Recommended For New Designs
      ZTI Zti; // Sentence Not Recommended For New Designs
      ZTG Ztg;
      ZWP Zwp; // Sentence Not Recommended For New Designs
      ZZU Zzu; // Sentence Not Recommended For New Designs


      QString ErrorMessage; // Filled when Parse returns FALSE
      QString LastSentenceIDParsed; // ID of the lst sentence successfully parsed
      QString LastSentenceIDReceived; // ID of the last sentence received, may not have parsed successfully
      QString PlainText; // A human-readable string of text that explains what just happened

      QString TalkerID;
      QString ExpandedTalkerID;

      virtual BOOL IsGood( void ) const;
      virtual BOOL Parse( void );
      //virtual void Serialize( CArchive& archive );

      NMEA0183& operator << ( const char *source );
      NMEA0183& operator >> ( QString& destination );
};

#endif // NMEA_0183_CLASS_HEADER
