#include "uartTranslater.h"
namespace pt = boost::property_tree;
Jobs::Jobs(boost::asio::io_service &io, std::vector<void *> &receivingSocketList, std::vector<void *> &sendingSocketList) : strand1(io),
                                                                                                                            strand2(io), strand3(io), strand4(io), strand5(io), timer1(io, boost::posix_time::milliseconds(5)), timer2(io, boost::posix_time::milliseconds(10)),
                                                                                                                            timer3(io, boost::posix_time::milliseconds(13)), timer4(io, boost::posix_time::milliseconds(16)), timer5(io, boost::posix_time::milliseconds(20)), rSocketList(receivingSocketList), sSocketList(sendingSocketList)
{
    InitParam();
    sockaddr_can addr;
    ifreq ifr;
    socketcan_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socketcan_fd < 0)
    {
        perror("ivactuator--> socket:");
        exit(-1);
    }
    strcpy(ifr.ifr_name, "can1");
    ioctl(socketcan_fd, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socketcan_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("ivactuator--> bind:");
        exit(-1);
    }

    rfilter[0].can_id = BCM_202;
    rfilter[0].can_mask = CAN_SFF_MASK;
    rfilter[1].can_id = EPS_401;
    rfilter[1].can_mask = CAN_SFF_MASK;
    rfilter[2].can_id = AVCU_XEP_86;
    rfilter[2].can_mask = CAN_SFF_MASK;
    rfilter[3].can_id = AVCU_TX2_F5;
    rfilter[3].can_mask = CAN_SFF_MASK;

    setsockopt(socketcan_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    timer1.async_wait(strand1.wrap(boost::bind(&Jobs::canSubscriber, this)));
    timer2.async_wait(strand2.wrap(boost::bind(&Jobs::socketSubscriber, this)));
    timer3.async_wait(strand3.wrap(boost::bind(&Jobs::canPublisher1, this)));
    timer4.async_wait(strand4.wrap(boost::bind(&Jobs::socketPublisher, this)));
    timer5.async_wait(strand5.wrap(boost::bind(&Jobs::canPublisher2, this)));
    std::cout << "jobs started." << std::endl;
}

void Jobs::canSubscriber()
{
    char ReadBuf[4096] = {0}; //+
    int nbytes = 0;
    can_frame frame;
    int frameSize = sizeof(frame); //+
    FD_ZERO(&rfd);
    FD_SET(socketcan_fd, &rfd);
    CanTimeout.tv_sec = 0;
    CanTimeout.tv_usec = 500000;
    if (select(socketcan_fd + 1, &rfd, 0, 0, &CanTimeout) == 0)
    {
        printf("Can Bus Timeout!");
        read_success_flag = 0;
        return;
    }
    else
    {
        read_success_flag = 1;
    }

    memset(&frame, 0, sizeof(frame));

    nbytes = read(socketcan_fd, &ReadBuf[0], 4096);

    if (nbytes < 0)
    {
        perror("ivactuator--> can raw socket read");
        return;
    }

    if (nbytes < sizeof(struct can_frame))
    {
        fprintf(stderr, "ivactuator--> read: incomplete CAN frame\n");
        return;
    }

    for (int i = 0; i < nbytes; i = i + frameSize)
    {
        memcpy(&frame, &ReadBuf[i], frameSize);

        if (BCM_202 == frame.can_id)
        {
            wheel_speed.wheelspeed_lr = (unsigned short)(frame.data[0] << 8 | frame.data[1]) - 3000;
            wheel_speed.wheelspeed_rr = (unsigned short)(frame.data[2] << 8 | frame.data[3]) - 3000;
            wheel_speed.pluse_mask = (unsigned char)frame.data[4];
            //gettimeofday(&receive_bcm202_last_time, NULL);
            double temp_di = 1024;
            double temp_speed = (wheel_speed.wheelspeed_lr + wheel_speed.wheelspeed_rr) * 0.5;

            wheel_speed.real_speed = (10 * temp_speed / temp_di) * 2 * 3.14 * 0.155;
            printf("speed:%f \n", wheel_speed.real_speed);
        }
        else if (EPS_401 == frame.can_id)
        {
            actuator_state.uisteerangle = (short)((frame.data[3] << 8) | frame.data[4]) - 1024;
            actuator_state.uisteerangle = actuator_state.uisteerangle / 8.3; // transfer to real speed
            //gettimeofday(&receive_eps401_last_time, NULL);

            printf("steering: %f \n ", actuator_state.uisteerangle);
        }
        else if (AVCU_XEP_86 == frame.can_id)
        {
            vehicle_state.sysstatus = (frame.data[0] >> 2) & 0x0F;
            printf("state:%d \n", vehicle_state.sysstatus);
            //gettimeofday(&receive_exp86_last_time, NULL);
        }
    }

    timer1.expires_at(timer1.expires_at() + boost::posix_time::milliseconds(TIMER1_PERIOD));
    timer1.async_wait(strand1.wrap(boost::bind(&Jobs::canSubscriber, this)));
}

void Jobs::socketSubscriber()
{
    char msg[2048] = {0};

    zmq_pollitem_t items[1];
    items[0].socket = rSocketList[0];
    items[0].events = ZMQ_POLLIN;

    //std::cout << "Polling Information ..." << std::endl;
    zmq_poll(items, 1, 0);

    if (items[0].revents & ZMQ_POLLIN)
    {
        int size = zmq_recv(rSocketList[0], msg, 2048, 0);
        if (size != -1)
        {

            std::stringstream ss;
            ss << msg;

            //          std::cout << "Got command Information." << std::endl;
            //          std::cout << ss.str() << std::endl;

           // Json::Value values;
           // Json::Reader reader;
        pt::ptree container;
        pt::read_json(ss.str(), container);
            if (true == container.empty())
            {
                vehicle_control.targetangle =(int)(container.get_child("Decision").get<double>("TargetSteeringAngleOut")* 8.3); // send real steering angle

                vehicle_control.targettorque = container.get_child("Decision").get<int>("TargetSpeedOut");
                //int accrequest=values["Decision"]["TargetAccReq"].asInt();//new
                //uint16_t workMode = values["Decision"]["TargetWorkMode"].asInt();
                //uint16_t steeringControlMode = values["Decision"]["TargetSteeringControlMode"].asInt();
                //uint16_t speedControlMode = values["Decision"]["TargetSpeedControlMode"].asInt();
                //            uint8_t red = values["Output"]["RedLight"].asInt();
                //            uint8_t yellow = values["Output"]["YellowLight"].asInt();
                //            std::cout << ss.str() <<std::endl;
                //            std::cout << values <<std::endl;

                std::cout << "Steering Angle Reference : " << vehicle_control.targetangle << std::endl;
                std::cout << "Vehicle Speed Reference : " << vehicle_control.targettorque << std::endl;
                // std::cout << "Work Mode : " << workMode << std::endl;
                //std::cout << "Steering Control Mode : " << steeringControlMode << std::endl;
                // std::cout << "Speed Control Mode : " << speedControlMode << std::endl;
                counter++;
                std::cout << "Counter : " << counter << std::endl;
                std::cout << std::endl;
                // soTargetSteeringAngle = dataSteering;
                //            velocity = static_cast<int>(dataSpeed*3.6*10);
                //dataSpeed = dataSpeed > 10.0?10.0:dataSpeed;
                //dataSpeed = dataSpeed < 0.0?0.0:dataSpeed;
                // soTargetSpeed = dataSpeed;
                //  soWorkMode = workMode;
                //            std::cout << soWorkMode << std::endl;
                // soSteeringControlMode = steeringControlMode;
                //soSpeedControlMode = speedControlMode;
                //            redLightStatus = red;
                //            yellowLightStatus = yellow;
                //            redLightStatus = 0;
                //            yellowLightStatus = 0;
            }
            else
            {
                std::cout << "Parse error  :(" << std::endl;
            }
        }
    }
    else
    {
        ;
    }

    //      usleep(SERIAL_SENDING_SLEEP);

    timer2.expires_at(timer2.expires_at() + boost::posix_time::milliseconds(TIMER2_PERIOD));
    timer2.async_wait(strand2.wrap(boost::bind(&Jobs::socketSubscriber, this)));
}

void Jobs::socketPublisher()
{
    //      std::cout << "PUBLISHING" << std::endl;
            pt::ptree values;

    values.get_child(KEY_ACTUATOR).put_value("VehicleSpeed",to_string(wheel_speed.real_speed)) ;;       // changed
    values.get_child(KEY_ACTUATOR).put_value("SteeringVelocity",to_string(siSteeringVelocity)) ;;       // changed
    values.get_child(KEY_ACTUATOR).put_value("TargetTorque",to_string(siTargetTorque)) ;;       // changed
    values.get_child(KEY_ACTUATOR).put_value("TargetBrakePressure",to_string(siTargetBrakePressure)) ;;       // changed
    values.get_child(KEY_ACTUATOR).put_value("BrakePressure",to_string(siBrakePressure)) ;;       // changed 
    values.get_child(KEY_ACTUATOR).put_value("SteeringTorqueVoltage",to_string(siSteeringTorqueVoltage)) ;;       // changed
    values.get_child(KEY_ACTUATOR).put_value("TargetSteeringAngle",to_string(siTargetSteeringAngle)) ;;       // changed
    values.get_child(KEY_ACTUATOR).put_value("TargetSteeringVelocity",to_string(siTargetSteeringVelocity)) ;;       // changed
    values.get_child(KEY_ACTUATOR).put_value("SteeringDAC_A",to_string(siSteeringDAC_A)) ;;       // changed
    values.get_child(KEY_ACTUATOR).put_value("SteeringDAC_B",to_string(siSteeringDAC_B)) ;;       // changed
    values.get_child(KEY_ACTUATOR).put_value("AcceleratorPedal",to_string(siAcceleratorPedal)) ;;       // changed


    //Json::FastWriter fastWriter;
    std::stringstream s2;
    pt::write_json(s2, values);
    std::string outstr = s2.str();
    //std::string jsonString = fastWriter.write(values);
    //jsonString.erase (std::remove(jsonString.begin(), jsonString.end(), '\n'), jsonString.end());
    zmq_send(sSocketList[0], outstr.c_str(), strlen(outstr.c_str()), 0);
    //      std::cout << "Sent: " << jsonString << std::endl;

    //      counter ++;
    //      std::cout << "Counter : " << counter << std::endl;

    timer4.expires_at(timer4.expires_at() + boost::posix_time::milliseconds(TIMER4_PERIOD));
    timer4.async_wait(strand4.wrap(boost::bind(&Jobs::socketPublisher, this)));
}

void Jobs::canPublisher1()
{
//SendBussinessControlData();
    SendControlData();
    SendSelfCheckFlag();
   // SendControlData();
    //SendBussinessControlData();

    timer3.expires_at(timer3.expires_at() + boost::posix_time::milliseconds(TIMER3_PERIOD));
    timer3.async_wait(strand3.wrap(boost::bind(&Jobs::canPublisher1, this)));
}

void Jobs::canPublisher2()
{
//SendBussinessControlData();
    //SendControlData();
    //SendSelfCheckFlag();
   // SendControlData();
    SendBussinessControlData();

    timer5.expires_at(timer5.expires_at() + boost::posix_time::milliseconds(TIMER5_PERIOD));
    timer5.async_wait(strand5.wrap(boost::bind(&Jobs::canPublisher2, this)));
}

void Jobs::InitParam() //这个有问题 不是wxb而是wbd
{
    carname = "wxb";
    is_ignore_faults = false;
    force_enter_selfdriving = false;
    force_open_top_lidar = false;
    top_lidar_status = false;

    read_success_flag = 1;
    bcm_202_success_flag = 1;
    eps_401_success_flag = 1;
    exp_86_success_flag = 1;
    actutorcommandselect = 0;
    force_enter_selfdriving = true;

    current_time = {0};
    receive_bcm202_last_time = {0};
    receive_eps401_last_time = {0};
    receive_exp86_last_time = {0};
    keyboardTimeout = {0, 100000};
    wheel_speed = {0};
    actuator_state = {0};
    vehicle_state = {0};
    task_control = {1};
    vehicle_control = {0, 0, 1, 3, 0, 0, 0};
    business_control = {0};
}

void Jobs::SendBussinessControlData()
{
    can_frame frame;
    memset(&frame, 0, sizeof(frame));
    int nbytes = 0;
    FD_ZERO(&keyfd);
    FD_SET(STDIN, &keyfd);
    keyboardTimeout.tv_sec = 0;
    keyboardTimeout.tv_usec = 100000;
    if (select(STDIN + 1, &keyfd, 0, 0, &keyboardTimeout) > 0)
    {
        char str[3];
        cin.getline(str, 3);
        business_control.openboxnum = atoi(str);
        cout << "openboxnum is " << business_control.openboxnum << endl;
    }
    else
	business_control.openboxnum = 0;

    frame.can_id = AVCU_TX2_F5;
    frame.can_dlc = 0x8;
    frame.data[0] = business_control.openboxnum & 0x1F;                                                  //开锁指令
    frame.data[1] = business_control.turnlightleftctrl & 0x01;                                           //
    frame.data[1] |= (business_control.turnlightrightctrl & 0x01) << 1;                                  //右转向灯控制
    frame.data[1] |= (business_control.headlightctrl & 0x01) << 2;                                       //前大灯控制
    frame.data[1] |= (business_control.brakelight & 0x01) << 3;                                          //刹车灯控制
    frame.data[1] |= (business_control.reversinglightctrl & 0x01) << 4;                                  //倒车灯控制
    frame.data[1] |= (business_control.doubleflashlightctrl & 0x01) << 5;                                //双闪灯控制
    frame.data[1] |= (business_control.taillightctrl & 0x01) << 6;                                       //示阔灯控制
    frame.data[1] |= (business_control.hornctrl & 0x01) << 7;                                            //喇叭控制
    frame.data[2] = (business_control.force_open_top_lidar || business_control.top_lidar_status) & 0x01; //顶激光雷达
    frame.data[2] |= 0x01 << 1;                                                                          //前激光雷达
    frame.data[2] |= (business_control.leftwheelbrakectrl & 0x01) << 2;                                  //左轮边制动器控制
    frame.data[2] |= (business_control.rightwheelbrakectrl & 0x01) << 3;                                 //右轮边制动器控制
    nbytes = write(socketcan_fd, &frame, sizeof(frame));
    if (nbytes != sizeof(frame))
    {
        cout << "Send AVCU_TX2_F5 Canbus Failed" << endl;
    }
    usleep(10 * 1000);
    //cout << "BusinessControlDataWell" << endl;
}

void Jobs::SendControlData()
{
    can_frame frame;
    memset(&frame, 0, sizeof(frame));
    int nbytes = 0;
    frame.can_id = AVCU_TX2_85;
    frame.can_dlc = 0x8;
    static unsigned int enter_selfdriving_count = 0;
    static unsigned int force_enter_selfdriving_flag = 0;
    if (force_enter_selfdriving == true)
    {
        actutorcommandselect = 0;
        if ((int)(vehicle_state.sysstatus) != 5)
        {
            enter_selfdriving_count++;
            if (enter_selfdriving_count < 60)
            {
                force_enter_selfdriving_flag = 0;
            }
            else if (enter_selfdriving_count >= 60 && enter_selfdriving_count < 70)
            {
                force_enter_selfdriving_flag = 1;
            }
            else if (enter_selfdriving_count >= 70 && enter_selfdriving_count < 130)
            {
                force_enter_selfdriving_flag = 0;
            }
            else
            {
                enter_selfdriving_count = 0;
            }
        }
        else
        {
            enter_selfdriving_count = 0;
        }
    }

    if (actutorcommandselect == 0) //0:自动;1:平行驾驶
    {
        frame.data[0] = (unsigned short)(vehicle_control.targetangle + 1024) / 256;
        frame.data[1] = (unsigned short)(vehicle_control.targetangle + 1024) % 256;
std::cout<<"send target angle:"<<vehicle_control.targetangle<<std::endl;
        frame.data[2] = 0x05; //(unsigned short)ivsteercontrol_msg.torque / 256;
        frame.data[3] = 0xDC; //(unsigned short)ivsteercontrol_msg.torque % 256;
        frame.data[4] = (unsigned short)vehicle_control.targettorque / 256;
        frame.data[5] = (unsigned short)vehicle_control.targettorque % 256;
        frame.data[6] = vehicle_control.actuatormode & 0x03;
        frame.data[6] |= (vehicle_control.shiftposition & 0x03) << 2;
        if (force_enter_selfdriving == false)
        {
            frame.data[6] |= (vehicle_control.autodrvmodeentry & 0x01) << 4; //进入自动驾驶模式指令
        }
        else
        {
            frame.data[6] |= (force_enter_selfdriving_flag & 0x01) << 4; //进入自动驾驶模式指令
        }

        frame.data[6] |= (vehicle_control.autodrvmodeexit & 0x01) << 5; //退出自动驾驶模式指令
        frame.data[6] |= (vehicle_control.sleepwakecmd & 0x03) << 6;    //休眠唤醒指令
        frame.data[7] = frame.data[0] ^ frame.data[1] ^ frame.data[2] ^ frame.data[3] ^ frame.data[4] ^ frame.data[5] ^ frame.data[6];
    }

    nbytes = write(socketcan_fd, &frame, sizeof(frame));
    if (nbytes != sizeof(frame))
    {
        cout << "Send AVCU_TX2_85 Canbus Failed" << endl;
    }
    usleep(10 * 1000);
   // cout << "SendControlDataWell" << endl;
}

void Jobs::SendSelfCheckFlag()
{
    can_frame frame;
    memset(&frame, 0, sizeof(frame));
    int nbytes = 0;
    frame.can_id = AVCU_TX2_84;
    frame.can_dlc = 0x8;
    frame.data[0] = 1 & 0x03; //??????????
    // frame.data[0] |= (SelfCheckState_Camera & 0x03)<<2;//?????????
    frame.data[0] |= (task_control.selfcheckstate_4g & 0x03) << 4;   //4G??????finished
    frame.data[0] |= (task_control.selfcheckstate_wifi & 0x03) << 6; //WiFi??????finished
    // frame.data[1] = (Reserved1 & 0x03);//??
    frame.data[1] |= (task_control.selfcheckstate_app & 0x03) << 2; //??APP??????finished
    // frame.data[1] |= (FailSafeLevel & 0x07)<<4;//??????
    frame.data[1] |= (task_control.isparallelmode & 0x01) << 7; //???????
    frame.data[2] = task_control.issafemode & 0x01;             //?????????????
    // frame.data[2] |= (MainTx2ResetCmd & 0x01)<<1;//?TX2????
    // frame.data[2] |= (SecTx2ResetCmd & 0x01)<<2;//?TX2????
    // frame.data[2] |= (MainTx2RecoveryCmd & 0x01)<<3;//?TX2????????
    // frame.data[2] |= (SecTx2RecoveryCmd & 0x01)<<4;//?TX2????????
    //frame.data[7] = LedDisplayCmd & 0x0F; //LED????
    nbytes = write(socketcan_fd, &frame, sizeof(frame));
    if (nbytes != sizeof(frame))
    {
        cout << "Send AVCU_TX2_84 Canbus Failed";
    }
    usleep(10 * 1000);
  //  cout << "SendSelfCheckFlagWell222";
}
