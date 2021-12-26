#ifndef DATAMSG_H
#define DATAMSG_H
#include <stdlib.h>

struct wheel_speed_msg
{
    unsigned short wheelspeed_lr;
    unsigned short wheelspeed_rr;
    unsigned char pluse_mask;
    double real_speed;
};

struct actuator_state_msg
{
    unsigned char epsstate;
    unsigned char realstrtorque;
    unsigned char eps_fault_code1;
    double uisteerangle;
    unsigned char anglecailstate;
    unsigned char eps_fault_code2;
};

struct vehiclestate_msg
{
    unsigned char vehicleselfcheckstate;
    unsigned char sysstatus;
    unsigned char shiftlvlposition;
    unsigned char forwardcrashswitchstate;
    unsigned char rearcrashswitchstate;
    unsigned char emergencystopstate;
    unsigned char remoteultavoidobsstate;
    unsigned char parrdrvultavoidobsstate;
    unsigned char chargingstopdrvstate;
};

struct control_msg
{
    short targetangle;
    unsigned short targettorque;
    unsigned char actuatormode;
    unsigned char shiftposition;
    unsigned char autodrvmodeentry;
    unsigned char autodrvmodeexit;
    unsigned char sleepwakecmd;
};

struct taskcontrol_msg
{
    unsigned char selfcheckstate_4g;
    unsigned char selfcheckstate_wifi;
    unsigned char selfcheckstate_app;
    unsigned char isparallelmode;
    unsigned char issafemode;
};

struct businesscontrol_msg
{
    unsigned char openboxnum;
    unsigned char turnlightleftctrl;
    unsigned char turnlightrightctrl;
    unsigned char headlightctrl;
    unsigned char brakelight;
    unsigned char reversinglightctrl;
    unsigned char doubleflashlightctrl;
    unsigned char taillightctrl;;
    unsigned char hornctrl;
    unsigned char force_open_top_lidar;
    unsigned char top_lidar_status;
    unsigned char leftwheelbrakectrl;
    unsigned char rightwheelbrakectrl;
};
#endif
