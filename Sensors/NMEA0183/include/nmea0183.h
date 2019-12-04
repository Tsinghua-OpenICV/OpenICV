#if ! defined( NMEA_0183_HEADER )

#define NMEA_0183_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**q
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1998, Samuel R. Blackburn
**
** $Workfile: nmea0183.h $
** $Revision: 6 $
** $Modtime: 10/13/98 6:37a $
*/

/*
** Updates : Maria Alwan & G�rald Dherbomez  
** Date :    14/09/2005
** purpose : use of Qt API instead of MFC
**           source code may be compatible with Windows, Linux and Mac
**           tested with Qt3.3.3 API under Windows
*/

#include <stdio.h>

#ifdef WIN32
#include <qt_windows.h>
#endif

#include <QString>
#include <QVector>
#include <QDateTime> 


/*
** Now include the math stuff for some calculations in the COORDINATE class
*/

#include <math.h>

/*
** Turn off the warning about precompiled headers, it is rather annoying
*/

#pragma warning( disable : 4699 )

#if ! defined( CARRIAGE_RETURN )
#define CARRIAGE_RETURN 0x0D
#endif

#if ! defined( LINE_FEED )
#define LINE_FEED       0x0A
#endif

typedef enum _NMEA0183_BOOLEAN
{
   NMEA_Unknown = 0,
   True,
   False
} NMEA0183_BOOLEAN;

typedef enum _leftright
{
   LR_Unknown = 0,
   Left,
   Right
} LEFTRIGHT;

typedef enum _eastwest
{
   EW_Unknown = 0,
   East,
   West
} EASTWEST;

typedef enum
{
   NS_Unknown = 0,
   North,
   South
} NORTHSOUTH;

typedef enum
{
   ReferenceUnknown = 0,
   BottomTrackingLog,
   ManuallyEntered,
   WaterReferenced,
   RadarTrackingOfFixedTarget,
   PositioningSystemGroundReference
} REFERENCE;

typedef enum
{
   CommunicationsModeUnknown         = 0,
   F3E_G3E_SimplexTelephone          = 'd',
   F3E_G3E_DuplexTelephone           = 'e',
   J3E_Telephone                     = 'm',
   H3E_Telephone                     = 'o',
   F1B_J2B_FEC_NBDP_TelexTeleprinter = 'q',
   F1B_J2B_ARQ_NBDP_TelexTeleprinter = 's',
   F1B_J2B_ReceiveOnlyTeleprinterDSC = 'w',
   A1A_MorseTapeRecorder             = 'x',
   A1A_MorseKeyHeadset               = '{',
   F1C_F2C_F3C_FaxMachine            = '|'
} COMMUNICATIONS_MODE;

typedef enum
{
   TransducerUnknown   = 0,
   AngularDisplacementTransducer = 'A',
   TemperatureTransducer         = 'C',
   LinearDisplacementTransducer  = 'D',
   FrequencyTransducer           = 'F',
   HumidityTransducer            = 'H',
   ForceTransducer               = 'N',
   PressureTransducer            = 'P',
   FlowRateTransducer            = 'R',
   TachometerTransducer          = 'T',
   VolumeTransducer              = 'V'
} TRANSDUCER_TYPE;

/*
** Misc Function Prototypes
*/

unsigned int HexValue( const char *hex_string );

QString expand_talker_id( const QString & );
QString Hex( unsigned int value );
QString talker_id( const QString& sentence );

#ifndef BOOL
#define BOOL bool
#endif

#ifndef TRUE
#define TRUE true
#endif

#ifndef FALSE
#define FALSE false
#endif

#include "nmea0183.hpp"

#endif // NMEA0183_HEADER
