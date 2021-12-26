#include <string>  
#include <msgpack.hpp>
//#include <TrackingBoxArray.h>
//#include <Map.h>
//#include <EgoPose.h>

//struct data_demo_type1 {
//    int buf1;
//    int buf2;
//    char buf3[11];
//    int buf4;
//    MSGPACK_DEFINE(buf1,buf2,buf3,buf4);
//};

struct data_demo_type1 {
    // ego vehicle
    double buf1;
    double buf2;
    double buf3;
    double buf4;
    double buf5;
    double buf6;
    double buf7;
    double buf8;
    double buf9;
    double buf10;
    double buf11;
    double buf12;
    // int buf5;
    // int buf6;
    //MSGPACK_DEFINE(buf1,buf2,buf3,buf4,buf5,buf6,buf7,buf8,buf9,buf10,buf11,buf12,buf13,buf14,buf15,buf16,buf17,buf18);
};

struct data_demo_type2 {
    // outputcontrol
    double acc;
    double steer;
    MSGPACK_DEFINE(acc, steer);
};