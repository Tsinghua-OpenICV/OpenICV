// donnees NMEA emises par l'ag132 sur la liaison serie
// revision le 23/08/2004 PhB et GD : trame_gga_dbl
// revision le 20/05/2005 Maria Alwan et Gerald Dherbomez : 5700

#ifndef STRUCTURE_GPS_H
#define STRUCTURE_GPS_H


/// POSITION WGS84
typedef struct
{
    double lat;       // radians
    double lon;       // radians
    double altitude;  // meters
    float sigma_lat;  // meters
    float sigma_lon;  // meters
    float sigma_alt;  // meters
    int32_t gps_week;    // GPS week number
    double gps_time;  // seconds
    int32_t quality;     // quality indicator of the position, referred to NMEA standard
    int32_t nb_sats;     // number of satellites used to compute the solution

} wgs84_position;

typedef struct
{
    wgs84_position position;
    int32_t time;
    int64_t timerange;
} TimestampedWgs84Position;

/// TRAME AVR
typedef struct
{
    int32_t H,Mi,S,Ms;
    float Yaw,Tilt,Range;
    int32_t ind_qualite;
    float pdop;
    int32_t nb_sat;
} trame_avr;

/// TRAME GGA, PRECISION FLOAT (30 cm en Lambert93)
typedef struct{
    int32_t H ,Mi,S;
    int32_t nb_sat,ind_qualite;
    float lon,age,lat,alt_msl,hdop,d_geoidal;
    int32_t Ms;//modif du 20040121 pour recupere les millisecondes
} trame_gga;

/// TRAME GGA, A UTILISER
typedef struct
{
    int32_t H,Mi,S,Ms;
    int32_t nb_sat,ind_qualite;
    float age,hdop;
    double lon,lat,alt_msl,d_geoidal;
    int8_t dir_lat,dir_lon;
    int32_t ref_station_ID;

    MSGPACK_DEFINE(H,Mi,S,Ms,nb_sat,ind_qualite,age,hdop,lon,lat,alt_msl,d_geoidal,dir_lat,dir_lon,ref_station_ID);

} trame_gga_dbl;

typedef struct DONNEES_GGA_
{
    trame_gga_dbl tramegga;
    int32_t t;
    DONNEES_GGA_()
    {
        t = 0;
        tramegga.alt_msl = 0;
        tramegga.d_geoidal = 0;
        tramegga.H = 0;
        tramegga.Mi = 0;
        tramegga.S = 0;
        tramegga.Ms = 0;
        tramegga.lat = 0;
        tramegga.lon = 0;
    }
} DONNEES_GGA;

/// TRAME GLL
typedef struct
{
    int32_t H,Mi,S,Ms;
    double lat,lon;
    int8_t dir_lat,dir_lon;
    int32_t valid_data;
} trame_gll;

/// TRAME GRS
typedef struct
{
    int32_t H,Mi,S,Ms;
    int32_t residus_status;
    float residus[12];
} trame_grs;

/// TRAME GSA
typedef struct
{
    int32_t mode_select, mode_result;
    int32_t SV_PRN[12];
    float pdop,hdop,vdop;
} trame_gsa;

/// TRAME GST
typedef struct
{
    float a,b,rms,phi,sigma_lat,sigma_lon,sigma_alt;
    int32_t H,Mi,S,Ms;
} trame_gst;

/// TRAME GSV
// Matrice SatellitesInView (36 satellites max) contient :
// en ligne : les 4 caracteristiques d'un satellite
// en colonne : SatelliteNumber, ElevationDegrees, AzimuthDegreesTrue, SignalToNoiseRatio
typedef struct{
    int32_t NumberOfSatellites;
    int32_t Totalmessages;
    int32_t SatellitesInView[36][4];
} trame_gsv;

/// TRAME HDT
typedef struct
{
    /// in degrees
    double DegreesTrue;
} trame_hdt;

/// TRAME MSS
typedef struct
{
    float SS,SNR;
    float beacon_freq;
    int32_t beacon_bitrate;
    int32_t channel;
} trame_mss;

/// TRAME RMC
typedef struct
{
    int32_t H,Mi,S,Ms,JJ,MM,AA;
    int32_t valid_data;
    double lat,lon;
    int8_t dir_lat, dir_lon;
    float vitesse;
    float track_true_north;
    float magnet_var;
    int8_t dir_magnet_var;
    int32_t mode;
} trame_rmc;

/// TRAME ROT
typedef struct
{
    double RateOfTurn;
    int32_t valid_data;
} trame_rot;

/// TRAME VTG
typedef struct
{
    /// meters per second [km/h]
    float v;
    float track_true_north, track_magnet_north;
} trame_vtg;

/// TRAME XTE
typedef struct
{
    int32_t valid_LCBlink;
    int32_t valid_LCCycleLock;
    float error;
    int32_t dir_to_steer;
} trame_xte;

/// TRAME_ZDA
typedef struct
{
    int32_t H,Mi,S,Ms,JJ,MM,AA;
    int32_t H_offset,Mi_offset;
} trame_zda;

/// GPS SYNCHRO FRAME
typedef struct
{
    trame_zda zda;                 // Data containing GPS time
    int64_t timeOffset;   // Time elapsed between PPS and ZDA
    int32_t ppsTime;           // timestamp of the last PPS received on the computer
    int32_t zdaTime;           // timestamp of the ZDA frame
} GpsSynchroFrame;


/// TRAME PTNLEV
typedef struct
{
    int32_t H,Mi,S,Ms;
    int32_t evt_nb;
} trame_ptnlev;

/// TRAME PTNLID
typedef struct
{
    int32_t machineID,productID;
    int32_t maj_FW_nb,min_FW_nb;
    int32_t JJ_FW,MM_FW,AA_FW;
} trame_ptnlid;

/// TRAME PTNLDG
typedef struct
{
    float SS,SNR,freq;
    int32_t bitrate,number,tracking;
    int32_t rtcm,ind_perf;
} trame_ptnldg;

/// TRAME PTNLSM
typedef struct
{
    int32_t ref_station_ID;
    int8_t message[30];
} trame_ptnlsm;

/// TRAME PTNL,GGK
typedef struct
{
    int32_t H,Mi,S,Ms,JJ,MM,AA;
    double lat,lon;
    int8_t dir_lat, dir_lon;
    int32_t ind_qualite,nb_sat;
    float dop;
    /// ellipsoidal height
    float eht;
} trame_ptnl_ggk;

/// TRAME PTNL,GGK_SYNC
typedef struct
{
    int32_t H,Mi,S,Ms,JJ,MM,AA;
    double lat,lon;
    int8_t dir_lat, dir_lon;
    int32_t ind_qualite,nb_sat;
    float dop;
    /// ellipsoidal height
    float eht;
} trame_ptnl_ggk_sync;

/// TRAME PTNL,PJK
typedef struct
{
    int32_t H,Mi,S,Ms,JJ,MM,AA;
    double northing,easting;
    int32_t ind_qualite,nb_sat;
    float dop;
    /// ellipsoidal height
    float eht;
} trame_ptnl_pjk;

/// TRAME PTNL,PJT
typedef struct
{
    int8_t coord_syst_name[30];
    int8_t projection_name[30];
} trame_ptnl_pjt;

/// TRAME PTNL,VGK
typedef struct
{
    int32_t H,Mi,S,Ms,JJ,MM,AA;
    double east_vect,north_vect,up_vect;
    int32_t ind_qualite,nb_sat;
    float dop;
} trame_ptnl_vgk;

/// TRAME PTNL,VHD
typedef struct
{
    int32_t H,Mi,S,Ms,JJ,MM,AA;
    float azimuth,delta_azimuth;
    float vert_angle,delta_vert_angle;
    float range,delta_range;
    int32_t ind_qualite,nb_sat;
    float pdop;
} trame_ptnl_vhd;

/// SPAN CPT SOLUTION STATUS
enum SolStatus
{
    SOL_COMPUTED = 0,
    INSUFFICIENT_OBS = 1,
    NO_CONVERGENCE = 2,
    SINGULARITY = 3,
    COV_TRACE = 4,
    TEST_DIST = 5,
    COLD_START = 6,
    V_H_LIMIT = 7,
    VARIANCE = 8,
    RESIDUALS = 9,
    DELTA_POS = 10,
    NEGATIVE_VAR = 11,
    INTEGRITY_WARNING = 13,
    IMU_UNPLUGGED = 17,
    PENDING = 18
};

/// SPAN CPT POSITION TYPE
enum PosType
{
    NONE = 0,
    FIXEDPOS = 1,
    FIXEDHEIGHT = 2,
    FLOATCONV = 4,
    WIDELANE = 5,
    NARROWLANE = 6,
    DOPPLER_VELOCITY = 8,
    SINGLE = 16,
    PSRDIFF = 17,
    WAAS = 18,
    PROPAGATED = 19,
    OMNISTAR = 20,
    L1_FLOAT = 32,
    IONOFREE_FLOAT = 33,
    NARROW_FLOAT = 34,
    L1_INT = 48,
    WIDE_INT = 49,
    NARROW_INT = 50,
    RTK_DIRECT_INS = 51,
    INS = 52,
    INS_PSRSP =53,
    INS_PSRDIFF =54,
    INS_RTKFLOAT = 55,
    INS_RTKFIXED = 56,
    INS_OMNISTAR = 57,
    INS_OMNISTAR_HP = 58,
    INS_OMNISTAR_XP = 59,
    OMNISTAR_HP = 64,
    OMNISTAR_XP = 65,
    CDGPS = 66
};

/// SPAN CPT Inertial Solution Status
enum INSStatus
{
    INS_INACTIVE = 0,
    INS_ALIGNING =1,
    INS_SOLUTION_NOT_GOOD = 2,
    INS_SOLUTION_GOOD = 3,
    INS_BAD_GPS_AGREEMENT = 4,
    INS_ALIGNMENT_COMPLETE = 5
};

/// TRAME BESTGPSPOSA
typedef struct
{
    SolStatus Status;
    PosType   posType;
    double    Lat;
    double    Lon;
    double    Hgt;
    float     Undulation;
    float     LatStd;
    float     LonStd;
    float     HgtStd;
    double    e;
    double    n;
    double    u;
} trame_bestgpsposa;

/// TRAME RAWIMUSA
typedef struct
{
    uint32_t Week;
    int32_t          reserved;
    double        Seconds;
    // FIXME: use a portable type instead of long double (which is 128-bit on Linux and 64-bit on Windows)
    /*long */double   ZAccel;
    /*long */double   YAccel;
    /*long */double   XAccel;
    /*long */double   ZGyro;
    /*long */double   YGyro;
    /*long */double   XGyro;
} trame_rawimusa;

/// TRAME INSPVAA
typedef struct
{
    uint32_t Week;
    int32_t          reserved;
    double        Seconds;
    double        Lat;
    double        Lon;
    double        Hgt;
    double        NorthVel;
    double        EastVel;
    double        UpVel;
    double        Roll;
    double        Pitch;
    double        Azimuth;
    INSStatus     Status;
    int32_t          reserved1;
    double        e;
    double        n;
    double        u;
} trame_inspvaa;

/// TRAME INSCOV
typedef struct
{
    uint32_t Week;
    int32_t   reserved;
    double Seconds;
    double PosCov[3][3];
    double AttCov[3][3];
    double VelCov[3][3];
} trame_inscov;

/// trame_bestgpsposa structure with timestamping
struct TimestampedBestgpsposaFrame
{
    trame_bestgpsposa frame;
    int32_t time;
    int64_t timerange;
};

/// trame_rawimusa structure with timestamping
struct TimestampedRawimusaFrame
{
    trame_rawimusa frame;
    int32_t time;
    int64_t timerange;
};

/// trame_inspvaa structure with timestamping
struct TimestampedInspvaaFrame
{
    trame_inspvaa frame;
    int32_t time;
    int64_t timerange;
};

/// trame_inscov structure with timestamping
struct TimestampedInscovFrame
{
    trame_inscov frame;
    int32_t time;
    int64_t timerange;
};

/// trame_gga structure with timestamping
struct TimestampedGgaFrame
{
    trame_gga_dbl frame;
    int32_t time;
    int64_t timerange;
};

/// trame_gst structure with timestamping
struct TimestampedGstFrame
{
    trame_gst frame;
    int32_t time;
    int64_t timerange;
};

/// trame_vtg structure with timestamping
struct TimestampedVtgFrame
{
    trame_vtg frame;
    int32_t time;
    int64_t timerange;
};

struct donnees_gps
{
    long  ind_qualite;
    double x, y;
    double lon, lat;
    float a, b, phi, sigma_lat, sigma_lon;
    int32_t time;
    double alt_msl, d_geoidal;
};


#endif // STRUCTURE_GPS_H
