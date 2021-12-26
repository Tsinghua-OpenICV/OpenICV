#if ! defined( NMEA_0183_CLASS_HEADER )

#define NMEA_0183_CLASS_HEADER

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

/*
** General Purpose Classes
*/

#include "SENTENCE.HPP"
#include "RESPONSE.HPP"
#include "LATLONG.HPP"
#include "LORANTD.HPP"
#include "MANUFACT.HPP"
#include "MLIST.HPP"
#include "OMEGAPAR.HPP"
#include "DECCALOP.HPP"
#include "RATIOPLS.HPP"
#include "RADARDAT.HPP"
#include "SATDAT.HPP"
#include "FREQMODE.HPP"
#include "WAYPTLOC.HPP" // Sentence Not Recommended For New Designs

/*
** Response Classes
*/

#include "AAM.HPP"
#include "ALM.HPP"
#include "APB.HPP"
#include "ASD.HPP"
#include "BEC.HPP"
#include "BOD.HPP"
#include "BWC.HPP"
#include "BWR.HPP"
#include "BWW.HPP"
#include "DBT.HPP"
#include "DCN.HPP"
#include "DPT.HPP"
#include "FSI.HPP"
#include "GDA.HPP" // Sentence Not Recommended For New Designs
#include "GDF.HPP" // Sentence Not Recommended For New Designs
#include "GDP.HPP" // Sentence Not Recommended For New Designs
#include "GGA.HPP"
#include "GLA.HPP" // Sentence Not Recommended For New Designs
#include "GLC.HPP"
#include "GLF.HPP" // Sentence Not Recommended For New Designs
#include "GLL.HPP"
#include "GLP.HPP" // Sentence Not Recommended For New Designs
#include "GOA.HPP" // Sentence Not Recommended For New Designs
#include "GOF.HPP" // Sentence Not Recommended For New Designs
#include "GOP.HPP" // Sentence Not Recommended For New Designs
#include "GSA.HPP"
#include "GST.HPP"
#include "GSV.HPP"
#include "GTD.HPP" // Sentence Not Recommended For New Designs
#include "GXA.HPP"
#include "GXF.HPP" // Sentence Not Recommended For New Designs
#include "GXP.HPP" // Sentence Not Recommended For New Designs
#include "HCC.HPP" // Sentence Not Recommended For New Designs
#include "HDG.HPP"
#include "HDM.HPP" // Sentence Not Recommended For New Designs
#include "HDT.HPP"
#include "HSC.HPP"
#include "IMA.HPP" // Sentence Not Recommended For New Designs
#include "LCD.HPP"
#include "MHU.HPP" // Sentence Not Recommended For New Designs
#include "MTA.HPP" // Sentence Not Recommended For New Designs
#include "MTW.HPP"
#include "MWV.HPP"
#include "OLN.HPP"
#include "OSD.HPP"
#include "P.HPP"
#include "RMA.HPP"
#include "RMB.HPP"
#include "RMC.HPP"
#include "ROT.HPP"
#include "RPM.HPP"
#include "RSA.HPP"
#include "RSD.HPP"
#include "RTE.HPP"
#include "SFI.HPP"
#include "STN.HPP"
#include "TEP.HPP" // Sentence Not Recommended For New Designs
#include "TRF.HPP"
#include "TTM.HPP"
#include "VBW.HPP"
#include "VDR.HPP"
#include "VHW.HPP"
#include "VLW.HPP"
#include "VPW.HPP"
#include "VTG.HPP"
#include "VWE.HPP" // Sentence Not Recommended For New Designs
#include "WCV.HPP"
#include "WDC.HPP" // Sentence Not Recommended For New Designs
#include "WDR.HPP" // Sentence Not Recommended For New Designs
#include "WNC.HPP"
#include "WPL.HPP"
#include "XDR.HPP"
#include "XTE.HPP"
#include "XTR.HPP"
#include "ZDA.HPP"
#include "ZFI.HPP" // Sentence Not Recommended For New Designs
#include "ZFO.HPP" 
#include "ZLZ.HPP" // Sentence Not Recommended For New Designs
#include "ZPI.HPP" // Sentence Not Recommended For New Designs
#include "ZTA.HPP" // Sentence Not Recommended For New Designs
#include "ZTE.HPP" // Sentence Not Recommended For New Designs
#include "ZTG.HPP"
#include "ZTI.HPP" // Sentence Not Recommended For New Designs
#include "ZWP.HPP" // Sentence Not Recommended For New Designs
#include "ZZU.HPP" // Sentence Not Recommended For New Designs

class NMEA0183
{
   private:

      SENTENCE m_Sentence;

      void m_Initialize( void );

   protected:

      std::vector<RESPONSE *> m_ResponseTable;

      void m_SetContainerPointers( void );

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
      GST Gst;
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

      std::string ErrorMessage; // Filled when Parse returns FALSE
      std::string LastSentenceIDParsed; // ID of the lst sentence successfully parsed
      std::string LastSentenceIDReceived; // ID of the last sentence received, may not have parsed successfully
      std::string PlainText; // A human-readable string of text that explains what just happened

      std::string TalkerID;
      std::string ExpandedTalkerID;

      virtual bool IsGood( void ) const;
      virtual bool Parse( void );
      virtual void SetSentence(const char * source);
      virtual void GetSentence(std::string& destination) const;

      NMEA0183& operator << ( const char *source );
      NMEA0183& operator >> ( std::string& destination );
};

#endif // NMEA_0183_CLASS_HEADER
