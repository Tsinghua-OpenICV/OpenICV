#ifndef UARTTRANSLATER_H
#define UARTTRANSLATER_H
#pragma once


//#include "canbus.h"
///
// for thread and timer:
#include <iostream>
#include <boost/asio.hpp>
#include <boost/asio/strand.hpp>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
//for json communication
// for zmq:
#include <zmq.h>
//#include <zmq_api.h>
// for json:
//#include <json/json.h>

//The lib file that zmq requires
#include <cmath>

#include "datamsg.h"

#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <math.h>
#include <net/if.h>
#include <semaphore.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>

#define AVCU_TX2_84 0x84
#define AVCU_TX2_85 0x85
#define AVCU_XEP_86 0x86
#define AVCU_TX2_F5 0xF5
#define AVCU_TX2_F6 0xF6
#define AVCU_XEP_469 0x469
#define ULT_611 0X611
#define ULT_612 0X612
#define ULT_613 0X613
#define ULT_614 0X614
#define ULT_615 0X615
#define ULT_616 0X616
#define ULT_617 0X617
#define BCM_111 0x111
#define BCM_112 0x112
#define BCM_202 0x202
#define EPS_401 0x401
#define BMS_501 0x501
#define BMS_502 0x502
#define BMS_503 0x503
#define BMS_504 0x504
#define BMS_505 0x505
#define BMS_521 0x521
#define BMS_522 0x522
#define BMS_523 0x523
#define AVCU_XEP_630 0x630
#define AVCU_XEP_631 0x631
#define BCM_640 0x640
#define BCM_641 0x641
#define ULT_0D0 0x0D0
#define ULT_0D1 0x0D1
#define ULT_0D2 0x0D2
#define ULT_0D3 0x0D3
#define ULT_0D5 0x0D5
#define AVCU_TX2_701 0x701
#define AVCU_TX2_702 0x702
#define AVCU_TX2_703 0x703
#define AVCU_XEP_741 0x741
#define BCM_742 0x742
#define DMCU_743 0x743
#define DMCU_321 0x321
#define DMCU_322 0x322
#define DMCU_323 0x323
#define SHAR_ID 7588

#define KEY_ACTUATOR "Actuator"

#define TIMER1_PERIOD 20
#define TIMER2_PERIOD 20
#define TIMER3_PERIOD 20
#define TIMER4_PERIOD 100
#define TIMER5_PERIOD 50

#define NO_SERIAL false

#define SERIAL_SENDING_SLEEP 1000

#define STDIN 0
namespace pt = boost::property_tree;

using namespace std;

class Jobs
{
public:
    explicit Jobs(boost::asio::io_service &io, std::vector<void *> &receivingSocketList, std::vector<void *> &sendingSocketList);
    ~Jobs(){};
    void canSubscriber();
    void socketSubscriber();
    void canPublisher1();
    void canPublisher2();
    void socketPublisher();

    //void recvCarInfoKernel();
    //void callback_sendthread();
    //void GetFaultCheckFlag();
    //unsigned int TimeDelayCheck(timeval time_now, timeval rec_info_time, const int max_wait_cnt,
    //const int recover_normal_cnt, const double info_period_time);
    void SendControlData();
    void SendSelfCheckFlag();
    void SendBussinessControlData();
    void InitParam();

private:
    boost::asio::io_context::strand strand1;
    boost::asio::io_context::strand strand2;
    boost::asio::io_context::strand strand3;
    boost::asio::io_context::strand strand4;
    boost::asio::io_context::strand strand5;
    boost::asio::deadline_timer timer1;
    boost::asio::deadline_timer timer2;
    boost::asio::deadline_timer timer3;
    boost::asio::deadline_timer timer4;
    boost::asio::deadline_timer timer5;

    std::vector<void *> rSocketList;
    std::vector<void *> sSocketList;

    //data to uart
    int32_t soWorkMode = 0,
            soSteeringControlMode = 0,
            soSpeedControlMode = 0;

    double soTargetSteeringAngle = 0,
           soTargetSteeringVelocity = 0,
           soTorqueSensorVoltage = 0,
           soTargetSpeed = 0,
           soTargetAcceleration = 0,
           soTargetTorque = 0,
           soTargetBrakePressure = 0;

    //data from uart
    //    uint8_t vehicleSpeed = 0;
    //    uint8_t steeringAngleIn = 0;
    //    uint8_t steeringAngleVelocity = 0;
    //    uint8_t rawSensor = 0;
    //    uint8_t pressureSensor = 0;
    //    uint8_t targetPressure = 0;
    //    uint8_t otherStatus = 0;

    double siVehicleSpeed = 0,
           siSteeringAngle = 0,
           siSteeringVelocity = 0,
           siTargetSteeringAngle = 0,
           siTargetSteeringVelocity = 0,
           siTargetTorque = 0,
           siTargetBrakePressure = 0,
           siBrakePressure = 0,
           siSteeringTorqueVoltage = 0,
           siSteeringDAC_A = 0,
           siSteeringDAC_B = 0,
           siAcceleratorPedal = 0;

    uint32_t counter = 0;

    int socketcan_fd;
    struct timeval CanTimeout, keyboardTimeout, receive_bcm202_last_time, receive_eps401_last_time, receive_exp86_last_time, current_time;
    struct can_filter rfilter[3];
    struct wheel_speed_msg wheel_speed;
    struct actuator_state_msg actuator_state;
    struct vehiclestate_msg vehicle_state;
    struct control_msg vehicle_control;
    struct taskcontrol_msg task_control;
    struct businesscontrol_msg business_control;
    fd_set rfd, keyfd;
    std::string carname;
    bool is_ignore_faults;
    bool force_enter_selfdriving;
    bool force_open_top_lidar;

    unsigned int read_success_flag;

    unsigned int bcm_202_success_flag;
    unsigned int eps_401_success_flag;
    unsigned int exp_86_success_flag;

    bool top_lidar_status;
    pt::ptree values;

    unsigned short actutorcommandselect;
};

#endif
