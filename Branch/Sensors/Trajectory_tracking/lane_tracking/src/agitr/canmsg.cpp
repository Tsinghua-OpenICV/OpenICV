//  @ Project : CAN
//  @ File Name : canmsg.cpp
//  @ Date : 2018/2/1
//  @ Author : 
//
//


#include "canmsg.h"
#include <stdio.h>


canmessages::canmessages(ros::NodeHandle nh)
{
  // ipHost="192.168.1.100";
  // portHost=4008;
  // ipTarget="192.168.1.178";
  // portTarget=4008;
  // ipHost="192.168.1.100";
  // portHost=4008;
  // ipTarget="192.168.1.178";
  // portTarget=4008;
  ipHost="192.168.110.1";
  portHost=4004;
  ipTarget="192.168.110.104";
  portTarget=4004;
  initUdp();  

  
  //receive data from topic ivsensorimu 
  pub_can = nh.advertise<tourancan::can>("touranmsg", 1000);
  accControl = 0.0;
  steeringAngleControl = 0.0;
  accActive = 0;
  steerActive = 0;
  steerOri = 0;
  flagaa = true;


  //ros::Rate loop_rate(10);
  //pub_markerArray = nh.advertise<visualization_msgs::MarkerArray>("markerArray", 1000);


}

canmessages::~canmessages()
{
	int ret1;
	uint8 sendBuf6ff[13];
	
        sendBuf6ff[5] = 0x00;
        sendBuf6ff[6] = 0x00;
        sendBuf6ff[7] = 0x00; 
        sendBuf6ff[8] = 0x00;
        sendBuf6ff[9] = 0x00;
        sendBuf6ff[10]= 0x00;
        sendBuf6ff[11]= 0x00;
        sendBuf6ff[12]= 0x00;
        ret1 = boostUdp->send_data(sendBuf6ff, 13);
}

void canmessages::initUdp(){
  // ROS_INFO("initUdp()..........");
  boost::asio::io_service io_service;
  boostUdp = new Boost_UDP(io_service,ipHost,portHost,ipTarget,portTarget);
  boostUdp->start_sock();
  // ROS_INFO("sock()..........");
}


//////发送配置信息/////////////////////
void canmessages::sendCanConfig() 
{
   

   uint8 sendBuf200[13];
   uint8 sendBuf6ff[13]; 


   

  while (1)
  {
        int accCan = (int)((accControl+7.22)/0.005);
        int steeringAngleCan  = (int)((steeringAngleControl/0.1));
        ROS_INFO ("accCan,%x",accCan);
        ROS_INFO ("steeringAngleCan,%x",steeringAngleCan);
        // ROS_INFO ("accCan&255,%x",(steeringAngleCan&7936)>>8);
 

        int ret1;

//=======================6ff============================
        memset(sendBuf6ff, 0, sizeof(sendBuf6ff));
        sendBuf6ff[0] = 0x08;
        sendBuf6ff[1] = 0x00;
        sendBuf6ff[2] = 0x00; 
        sendBuf6ff[3] = 0x06;
        sendBuf6ff[4] = 0xff;
//////////////data segment/////////////
        sendBuf6ff[5] = (steerOri<<5)|(steerActive<<2)|(accActive);
        sendBuf6ff[6] = accCan&255;
        sendBuf6ff[7] = (accCan&1792)>>8; 
        sendBuf6ff[8] = 0x00;
        sendBuf6ff[9] = 0x10;
        sendBuf6ff[10]= steeringAngleCan&255;
        sendBuf6ff[11]= (steeringAngleCan&7936)>>8;
        sendBuf6ff[12]= 0x00;

        ROS_INFO ("sendBuf6ff[5],%x",sendBuf6ff[5]);
        ROS_INFO ("sendBuf6ff[6],%x",sendBuf6ff[6]);
        ROS_INFO ("sendBuf6ff[7],%x",sendBuf6ff[7]);
        ROS_INFO ("sendBuf6ff[10],%x",sendBuf6ff[10]);
        ROS_INFO ("sendBuf6ff[11],%x",sendBuf6ff[11]);

        ret1 = boostUdp->send_data(sendBuf6ff, 13);
  }
       
    

}



///////////////////////////////


void canmessages::run(float acclerationIn, float steeringAngleIn)
{   

    if (acclerationIn < -7.0){
      acclerationIn = -7.0;
      accActive = 1;
    }
    else if(acclerationIn > 3.0){
      acclerationIn = 3.0;
      accActive = 1;
    }
    else
    {
     accActive = 1;

    }

      
       
    if (steeringAngleIn<0.0){
      steeringAngleIn = -steeringAngleIn;
      steerOri = 1;
      steerActive = 1;
    }
    else{
      steerOri = 0;
      steerActive = 1;
    }

    if (steeringAngleIn>487.0)
      steeringAngleIn = 487.0;

    accControl = acclerationIn;
    steeringAngleControl = steeringAngleIn; 
    // send thread
    if (flagaa){


    
    boost::function<void()> send = boost::bind(&canmessages::sendCanConfig, this);
    boost::thread sendCanConfigThread(send);  

  // receive thread for LRR
  boost::function<void()> recLrr = boost::bind(&canmessages::runCanMain, this);
  boost::thread runCanMainThread(recLrr); 
  flagaa = false;
    }
  // std::cerr<<"accControl,"<<accControl<<std::endl;
  // std::cerr<<"steeringAngleControl,"<<steeringAngleControl<<std::endl;
  ROS_INFO("run()..........");
}

void canmessages::runCanMain()
{
  int ret;
  bool isReady;

  while (ros::ok())
  { 
    
    memset(udpBuffer, 0, sizeof(udpBuffer));
    ret = boostUdp->receive_data(udpBuffer);
    if (ret > 0 && ret%13 == 0)
    {
      
      //printf("# received %d frames \n", ret/13);
      
	//shuju jie bao
      isReady = recCanDate(ret); //一次接收一个数据包，一个包包括很多帧数据，ret/13是帧数，处理接受 过来的数据 如果所有帧处理完毕，则isReady=True，则publish目标 


    }
    else
    {
      printf("Error: have not received LRR data, or byte number is wrong!\n");
      break;
    }
  }

}



bool canmessages::recCanDate(uint16 num_buf) 
{
  uint16 i, j;
  int num=0;
  int numall=0;
  uint16 num_frame = num_buf / 13;//数据帧数
  //lrr 
  int flag = 0;
  stCANMsg frame;
  
  for (i = 0; i < num_frame; i++)
  {
  //取第i帧
    frame.ID = ((uint16)(udpBuffer[i*13+3] << 8) + udpBuffer[i*13+4]);
    //// ROS_INFO("#id: %x", frame.ID);
    for (j = 0; j < 8; j++)
    {
      frame.data[j] = udpBuffer[i*13 + j + 5];//取第i帧内所有的数据
    }
    
  //////////////ESP_02/////////// 
    if(frame.ID==0x101)
    {
      CANbusESP_02(&frame);
    }
  //////////////ESP_05/////////// 
    if(frame.ID==0x106)
    {
      CANbusESP_05(&frame);
    }
    //////////////ESP_10/////////// 
    if(frame.ID==0x116)
    {
      CANbusESP_10(&frame);
    } 
   //////////////ESP_19/////////// 
    if(frame.ID==0xB2)
    {
      CANbusESP_19(&frame);
    } 
   //////////////ESP_21/////////// 
    if(frame.ID==0xFD)
    {
      CANbusESP_21(&frame);
    } 
  //////////////ESP_33/////////// 
    if(frame.ID==0x1AB)
    {
      CANbusESP_33(&frame);
    } 
  //////////////Gateway_73/////////// 
    if(frame.ID==0x3DC)
    {
      CANbusGateway_73(&frame);
    }
  //////////////Getriebe_11///////////
    if(frame.ID==0xAD)
    {
      CANbusGetriebe_11(&frame);
    }
  //////////////HCA_01///////////
    if(frame.ID==0x126)
    {
      CANbusHCA_01(&frame);
    }
  //////////////LH_EPS_03///////////
    if(frame.ID==0x9F)
    {
      CANbusLH_EPS_03(&frame);
    }
  //////////////LWI_01///////////
    if(frame.ID==0x86)
    {
      CANbusLWI_01(&frame);
    }
  //////////////Motor_14///////////
    if(frame.ID==0x3BE)
    {
      CANbusMotor_14(&frame);
    }
  //////////////Motor_20///////////
    if(frame.ID==0x121)
    {
      CANbusMotor_20(&frame);
    }
  //////////////Motor_Code_01///////////
    if(frame.ID==0x641)
    {
      CANbusMotor_Code_01(&frame);
    }
    /////////////TSK_06///////////
    if(frame.ID==0x120)
    {
      CANbusTSK_06(&frame);
    }
  /////////////ACC///////////
  if(frame.ID==0x30C)
    {
      CANbusACC_02(&frame);
    }
  if(frame.ID==0x122)
    {
      CANbusACC_06(&frame);
    }
  if(frame.ID==0x12E)
    {
      CANbusACC_07(&frame);
    }
  /////////////OBD_01///////////
  if(frame.ID==0x391)
    {
      CANbusOBD_01(&frame);
    }
  /////////////PLA_01///////////
  if(frame.ID==0x130)
    {
      CANbusPLA_01(&frame);
    } 
  /////////////EPB_01///////////
  if(frame.ID==0x130)
    {
      CANbusEPB_01(&frame);
    }
/////////////////////////////

  // publish canmsg
    pubLrrRadarData();

  }

  //// ROS_INFO("mumall=:%d",numall);
  return true;

}

void canmessages::CANbusLWI_01(stCANMsg *frame)
{
   LWI_01.LWI_Sensorstatus=0;
   LWI_01.LWI_StrWhlAngleSt=0;
   LWI_01.LWI_StrWhlAngleSize=0;
   LWI_01.LWI_StrWhlAngleDrt=0;
   LWI_01.LWI_StrWhlSpeedDrt=0;
   LWI_01.LWI_StrWhlSpeedSize=0;
   
   uint8 *pData = frame->data;
   int16 temp,temp1;
   
   temp =0; 
   temp=(short)(pData[1]&0x10);
   temp=temp>>4;
   LWI_01.LWI_Sensorstatus=temp;
   
    temp =0; 
   temp=(short)(pData[1]&0x80);
   temp=temp>>7;
   LWI_01.LWI_StrWhlAngleSt=temp;
   
   
   temp =0; 
   temp1=0;
   temp=(short)(pData[2]&0xFF);
   temp1=(short)(pData[3]&0x1F);
   temp1=temp1<<8;
   temp1+=temp;
   LWI_01.LWI_StrWhlAngleSize=temp1*0.1;
   
   temp =0; 
   temp=(short)(pData[3]&0x20);
   temp=temp>>5;
   LWI_01.LWI_StrWhlAngleDrt=temp;
   
   temp =0; 
   temp=(short)(pData[3]&0x40);
   temp=temp>>6;
   LWI_01.LWI_StrWhlSpeedDrt=temp;
   
   temp =0; 
   temp1=0;
   temp=(short)(pData[3]&0x80);
   temp=temp>>7;
   temp1=(short)(pData[4]&0xFF);
   temp1=temp1<<1;
   temp1+=temp;
   LWI_01.LWI_StrWhlSpeedSize=temp1*5;
   
    // // ROS_INFO("LWI_Sensorstatus:%d",LWI_01.LWI_Sensorstatus);
    // // ROS_INFO("LWI_StrWhlAngleSt:%d",LWI_01.LWI_StrWhlAngleSt);
    // // ROS_INFO("LWI_StrWhlAngleSize:%f",LWI_01.LWI_StrWhlAngleSize);
    // // ROS_INFO("LWI_StrWhlAngleDrt:%d",LWI_01.LWI_StrWhlAngleDrt);
    // // ROS_INFO("LWI_StrWhlSpeedDrt:%d",LWI_01.LWI_StrWhlSpeedDrt);
    // // ROS_INFO("LWI_StrWhlSpeedSize:%f",LWI_01.LWI_StrWhlSpeedSize);
	
}

//==================================================================

//===================================================================

void canmessages::CANbusGetriebe_11(stCANMsg *frame)
{
  Getriebe_11.Stalls=0;
  Getriebe_11.TargetStalls=0;
  uint8 *pData = frame->data;
  int16 temp;
  
  temp=0;
  temp=(short)(pData[5]&0x3C);
  temp=temp>>2;
  Getriebe_11.Stalls=temp;
  
  temp=0;
  temp=(short)(pData[7]&0xF0);
  temp=temp>>4;
  Getriebe_11.TargetStalls=temp;
  
  // // ROS_INFO("Stalls:%f",Getriebe_11.Stalls);
  // // ROS_INFO("TargetStalls:%f",Getriebe_11.TargetStalls);
}
//===================================================================
void canmessages::CANbusGateway_73(stCANMsg *frame)
{
  Gateway_73.EngineSpeedState=0;
  Gateway_73.EngineSpeed=0;
  uint8 *pData = frame->data;
  int16 temp,temp1;
  
  temp=0;
  temp=(short)(pData[1]&0x01);
  Gateway_73.EngineSpeedState=temp;
  
  temp=0;
  temp=(short)(pData[2]&0xFF);
  temp1=0;
  temp1=(short)(pData[3]&0xFF);
  temp1=temp1<<8;
  temp1+=temp;
  Gateway_73.EngineSpeed=temp1*0.25;
  
  // // ROS_INFO("EngineSpeedState:%d",Gateway_73.EngineSpeedState);
  // // ROS_INFO("EngineSpeed:%f",Gateway_73.EngineSpeed);
}
//===================================================================
void canmessages::CANbusESP_33(stCANMsg *frame)
{
  ESP_33.ACCSignalContinuity=0;
  uint8 *pData = frame->data;
  int16 temp;
  
  temp=0;
  temp=(short)(pData[4]&0x40);
  temp=temp>>6;
  ESP_33.ACCSignalContinuity=temp;
  // // ROS_INFO("ACCSignalContinuity:%d",ESP_33.ACCSignalContinuity);
}
//===================================================================
void canmessages::CANbusTSK_06(stCANMsg *frame)
{
  TSK_06.TSK_status=0;
  uint8 *pData = frame->data;
  int16 temp;
  
  temp=0;
  temp=(short)(pData[3]&0x7);
  TSK_06.TSK_status=temp;
  // // ROS_INFO("TSK_status:%f",TSK_06.TSK_status);
}
//===================================================================
void canmessages::CANbusEPB_01(stCANMsg *frame)
{
  EPB_01.EPBFaultStatus=0;
  EPB_01.EPBSwitch=0;
  EPB_01.EPBSwitchState=0;
  EPB_01.EPB_PressingForce=0;
  EPB_01.EPB_Status=0;
  uint8 *pData = frame->data;
  int16 temp,temp1;
  
  temp=0;
  temp=(short)(pData[6]&0xc);
  temp=temp>>2;
  EPB_01.EPBFaultStatus=temp;

  temp=0;
  temp=(short)(pData[6]&0x30);
  temp=temp>>4;
  EPB_01.EPBSwitch=temp;

  temp=0;
  temp=(short)(pData[6]&0x40);
  temp=temp>>6;
  EPB_01.EPBSwitchState=temp;

  temp=0;
  temp=(short)(pData[7]&0x1f);
  EPB_01.EPB_PressingForce=temp;

  temp=0;
  temp=(short)(pData[7]&0x60);
  temp=temp>>5;
  EPB_01.EPB_Status=temp;

  // // ROS_INFO("EPBFaultStatus:%f",EPB_01.EPBFaultStatus);
  // // ROS_INFO("EPBSwitch:%f",EPB_01.EPBSwitch);
  // // ROS_INFO("EPBSwitchState:%d",EPB_01.EPBSwitchState);
  // // ROS_INFO("EPB_PressingForce:%f",EPB_01.EPB_PressingForce);
  // // ROS_INFO("EPB_Status:%f",EPB_01.EPB_Status);
}
//===================================================================
void canmessages::CANbusESP_21(stCANMsg *frame)
{
  ESP_21.Speed=0;
  ESP_21.ESP_SystemStatus=0;
  ESP_21.SpeedState=0;
  uint8 *pData = frame->data;
  int16 temp,temp1;

  temp=0;
  temp=(short)(pData[4]&0xFF);
  temp1=0;
  temp1=(short)(pData[5]&0xFF);
  temp1=temp1<<8;
  temp1+=temp;
  ESP_21.Speed=temp1*0.01;

  temp=0;
  temp=(short)(pData[6]&0x04);
  temp1=temp>>2;
  ESP_21.ESP_SystemStatus=temp;

  temp=0;
  temp=(short)(pData[6]&0x80);
  temp1=temp>>7;
  ESP_21.SpeedState=temp;

  // // ROS_INFO("Speed:%f",ESP_21.Speed);
  // // ROS_INFO("ESP_SystemStatus:%d",ESP_21.ESP_SystemStatus);
  // // ROS_INFO("SpeedState:%d",ESP_21.SpeedState);
}
//===================================================================
void canmessages::CANbusESP_19(stCANMsg *frame)
{
  ESP_19.LF_WhlSpd=0;
  ESP_19.LR_WhlSpd=0;
  ESP_19.RF_WhlSpd=0;
  ESP_19.RR_WhlSpd=0;
  uint8 *pData = frame->data;
  int16 temp,temp1;
  
  temp=0;
  temp=(short)(pData[4]&0xFF);
  temp1=0;
  temp1=(short)(pData[5]&0xFF);
  temp1=temp1<<8;
  temp1+=temp;
  ESP_19.LF_WhlSpd=temp1*0.0075;
  
  temp=0;
  temp=(short)(pData[0]&0xFF);
  temp1=0;
  temp1=(short)(pData[1]&0xFF);
  temp1=temp1<<8;
  temp1+=temp;
  ESP_19.LR_WhlSpd=temp1*0.0075;
  
  temp=0;
  temp=(short)(pData[6]&0xFF);
  temp1=0;
  temp1=(short)(pData[7]&0xFF);
  temp1=temp1<<8;
  temp1+=temp;
  ESP_19.RF_WhlSpd=temp1*0.0075;

  temp=0;
  temp=(short)(pData[2]&0xFF);
  temp1=0;
  temp1=(short)(pData[3]&0xFF);
  temp1=temp1<<8;
  temp1+=temp;
  ESP_19.RR_WhlSpd=temp1*0.0075;

  // // ROS_INFO("LF_WhlSpd:%f",ESP_19.LF_WhlSpd);
  // // ROS_INFO("LR_WhlSpd:%f",ESP_19.LR_WhlSpd);
  // // ROS_INFO("RF_WhlSpd:%f",ESP_19.RF_WhlSpd);
  // // ROS_INFO("RR_WhlSpd:%f",ESP_19.RR_WhlSpd);
}
//===================================================================
//==================================================================
void canmessages::CANbusMotor_20(stCANMsg *frame)
{
  Motor_20.aPedalPercent=0;
  Motor_20.aPedalPercentSt=0; 
  Motor_20.ThrottleGradientSize=0;
  Motor_20.ThrottleGradientPN=0;
  Motor_20.EngNeutralTorque=0; 
  uint8 *pData = frame->data;
  int16 temp,temp1;
 
  temp =0;
  temp1=0;
  temp=(short)(pData[1]&0xF0);
  temp=temp>>4;
  temp1=(short)(pData[2]&0x0F);
  temp1=temp1<<4;
  temp1+=temp;
  Motor_20.aPedalPercent=temp1*0.4;

  temp =0;
  temp1=0;
  temp=(short)(pData[2]&0xE0);
  temp=temp>>5;
  temp1=(short)(pData[3]&0x1F);
  temp1=temp1<<3;
  temp1+=temp;
  Motor_20.ThrottleGradientSize=temp1*25;

  temp =0;
  temp=(short)(pData[2]&0x10);
  temp=temp>>4;
  Motor_20.aPedalPercentSt=temp;

  temp =0;
  temp=(short)(pData[3]&0x20);
  temp=temp>>5;
  Motor_20.ThrottleGradientPN=temp;

  temp =0;
  temp=(short)(pData[4]&0x20);
  temp=temp>>5;
  Motor_20.EngNeutralTorque=temp;

  // // ROS_INFO("aPedalPercent:%f",Motor_20.aPedalPercent);
  // // ROS_INFO("aPedalPercentSt:%d",Motor_20.aPedalPercentSt);
  // // ROS_INFO("ThrottleGradientSize:%f",Motor_20.ThrottleGradientSize);
  // // ROS_INFO("ThrottleGradientPN:%d",Motor_20.ThrottleGradientPN);
  // // ROS_INFO("EngNeutralTorque:%d", Motor_20.EngNeutralTorque);
}  

//==================================================================
void canmessages::CANbusMotor_14(stCANMsg *frame)
{
  Motor_14.BrakeSwitch=0;
  uint8 *pData = frame->data;
  int16 temp;

  temp =0; 
  temp=(short)(pData[3]&0x40);
  temp=temp>>6;
  Motor_14.BrakeSwitch=temp;

  // ROS_INFO("BrakeSwitch:%d",Motor_14.BrakeSwitch);
}

//==================================================================
void canmessages::CANbusLH_EPS_03(stCANMsg *frame)
{
  LH_EPS_03.EPSRxHCA_Status=0;
  LH_EPS_03.EPS_StrWhlTorque=0;
  LH_EPS_03.EPS_StrWhlTorqueDrt=0;
  LH_EPS_03.EPS_StrWhlTorqueSt=0;

  uint8 *pData = frame->data;
  int16 temp,temp1;

  temp=0;
  temp=(short)(pData[4]&0x0F);
  LH_EPS_03.EPSRxHCA_Status=temp;

  temp=0;
  temp=(short)(pData[5]&0xFF);
  temp1=0;
  temp1=(short)(pData[6]&0x03);
  temp1=temp1<<8;
  temp1+=temp;
  LH_EPS_03.EPS_StrWhlTorque=temp1*0.01;

  temp=0;
  temp=(short)(pData[6]&0x80);
  temp=temp>>7;
  LH_EPS_03.EPS_StrWhlTorqueDrt=temp;

  temp=0;
  temp=(short)(pData[6]&0x40);
  temp=temp>>6;
  LH_EPS_03.EPS_StrWhlTorqueSt=temp;

  // // ROS_INFO("EPSRxHCA_Status:%f",LH_EPS_03.EPSRxHCA_Status);
  // // ROS_INFO("EPS_StrWhlTorque:%f",LH_EPS_03.EPS_StrWhlTorque);
  // // ROS_INFO("EPS_StrWhlTorqueDrt:%d",LH_EPS_03.EPS_StrWhlTorqueDrt);
  // // ROS_INFO("EPS_StrWhlTorqueSt:%d",LH_EPS_03.EPS_StrWhlTorqueSt);

}
//==================================================================
void canmessages::CANbusHCA_01(stCANMsg *frame)
{
  HCA_01.ExpStrWhlTorque=0;
  HCA_01.HCATranCycle=0;
  HCA_01.ExpStrWhlTorqueDrt=0;
  HCA_01.HCA_Status=0;
  uint8 *pData = frame->data;
  int16 temp,temp1;

  temp=0;
  temp=(short)(pData[2]&0xFF);
  temp1=0;
  temp1=(short)(pData[3]&0x01);
  temp1=temp1<<8;
  temp1+=temp;
  HCA_01.ExpStrWhlTorque=temp1*0.01;


  temp=0;
  temp=(short)(pData[3]&0x40);
  temp=temp>>6;
  HCA_01.HCATranCycle=temp;

  temp=0;
  temp=(short)(pData[3]&0x80);
  temp=temp>>7;
  HCA_01.ExpStrWhlTorqueDrt=temp;

  temp=0;
  temp=(short)(pData[4]&0x0F);
  HCA_01.HCA_Status=temp;

  // // ROS_INFO("ExpStrWhlTorque:%f",HCA_01.ExpStrWhlTorque);
  // // ROS_INFO("HCATranCycle:%d",HCA_01.HCATranCycle);
  // // ROS_INFO("ExpStrWhlTorqueDrt:%d",HCA_01.ExpStrWhlTorqueDrt);
  // // ROS_INFO("HCA_Status:%f",HCA_01.HCA_Status);
}
//==================================================================
void canmessages::CANbusMotor_Code_01(stCANMsg *frame)
{
  Motor_Code_01.EngTorqueCoefficient=0;
  uint8 *pData = frame->data;
  int16 temp;

  temp=0;
  temp=(short)(pData[1]&0x30);
  temp=temp>>4;
  Motor_Code_01.EngTorqueCoefficient=temp;

  // // ROS_INFO("EngTorqueCoefficient:%f",Motor_Code_01.EngTorqueCoefficient);
}
//==================================================================
void canmessages::CANbusOBD_01(stCANMsg *frame)
{
  OBD_01.ThtottlePosition=0;
  OBD_01.aPedalPosition=0;
  uint8 *pData = frame->data;
  int16 temp;

  temp=0;
  temp=(short)(pData[2]&0xFF);
  OBD_01.ThtottlePosition=temp*0.392;
 
  temp=0;
  temp=(short)(pData[5]&0xFF);
  OBD_01.aPedalPosition=temp*0.392;

  // // ROS_INFO("ThtottlePosition:%f",OBD_01.ThtottlePosition);
  // // ROS_INFO("aPedalPosition:%f",OBD_01.aPedalPosition);
}
//==================================================================

void canmessages::CANbusPLA_01(stCANMsg *frame)
{
  PLA_01.PLA_CRC=0;
  PLA_01.PLA_BZ=0;
  PLA_01.PLABrkRqtSt=0;
  PLA_01.PLAExpStrWhlAngle=0;//期望方向盘转角
  PLA_01.PLAExpStrWhlAngleDrt=0;
  PLA_01.PLARequestStatus=0;
  PLA_01.PLABrkTorque=0;
  PLA_01.PLABrkDeceleration=0;
  PLA_01.PLABrkEnable=0;
  PLA_01.BrkTrqAndDeceSwt=0;
  PLA_01.PLAParking=0;
  PLA_01.PLAPrkDistance=0;
  PLA_01.PLASignalTxCyclic=0;
  uint8 *pData = frame->data;
  int16 temp,temp1,temp2;

  temp=0;
  temp=(short)(pData[0]&0xFF);
  PLA_01.PLA_CRC=temp;
 
  temp=0;
  temp=(short)(pData[1]&0x0F);
  PLA_01.PLA_BZ=temp;

  temp=0;
  temp=(short)(pData[1]&0xF0);
  temp=temp>>4;
  PLA_01.PLABrkRqtSt=temp;

  temp=0;
  temp=(short)(pData[2]&0xFF);
  temp1=0;
  temp1=(short)(pData[3]&0x1F);
  temp1=temp1<<8;
  temp1+=temp;
  PLA_01.PLAExpStrWhlAngle=temp1*0.1;

  temp=0;
  temp=(short)(pData[3]&0x80);
  temp=temp>>7;
  PLA_01.PLAExpStrWhlAngleDrt=temp;

  temp=0;
  temp=(short)(pData[4]&0x0F);
  PLA_01.PLARequestStatus=temp;

  temp=0;
  temp=(short)(pData[4]&0xF0);
  temp=temp>>4;
  temp1=0;
  temp1=(short)(pData[5]&0xFF);
  temp1=temp1<<4;
  temp1+=temp;
  temp2=0;
  temp2=(short)(pData[5]&0x01);
  temp2=temp2<<12;
  temp2+=temp1;
  PLA_01.PLABrkTorque=temp2*4;
  
  temp=0;
  temp=(short)(pData[4]&0xF0);
  temp=temp>>4;
  temp1=0;
  temp1=(short)(pData[5]&0x07);
  temp1=temp1<<4;
  temp1+=temp;
  PLA_01.PLABrkDeceleration=temp1*0.1;

  temp=0;
  temp=(short)(pData[5]&0x08);
  temp=temp>>3;
  PLA_01.PLABrkEnable=temp;

  temp=0;
  temp=(short)(pData[6]&0x04);
  temp=temp>>2;
  PLA_01.BrkTrqAndDeceSwt=temp;

  temp=0;
  temp=(short)(pData[6]&0x08);
  temp=temp>>3;
  PLA_01.PLAParking=temp;

  temp=0;
  temp=(short)(pData[6]&0xF0);
  temp=temp>>4;
  temp1=0;
  temp1=(short)(pData[7]&0x7F);
  temp1=temp1<<4;
  temp1+=temp;
  PLA_01.PLAPrkDistance=temp1*0.01;

  temp=0;
  temp=(short)(pData[7]&0x80);
  temp=temp>>7;
  PLA_01.PLASignalTxCyclic=temp;

  // // ROS_INFO("PLA_CRC:%f",PLA_01.PLA_CRC);
  // // ROS_INFO("PLA_BZ:%f",PLA_01.PLA_BZ);
  // // ROS_INFO("PLABrkRqtSt:%d",PLA_01.PLABrkRqtSt);
  // // ROS_INFO("PLAExpStrWhlAngle:%f",PLA_01.PLAExpStrWhlAngle);

  // // ROS_INFO("PLAExpStrWhlAngleDrt:%d",PLA_01.PLAExpStrWhlAngleDrt);
  // // ROS_INFO("PLARequestStatus:%f",PLA_01.PLARequestStatus);
  // // ROS_INFO("PLABrkTorque:%f",PLA_01.PLABrkTorque);
  // // ROS_INFO("PLABrkDeceleration:%f",PLA_01.PLABrkDeceleration);

  // // ROS_INFO("PLABrkEnable:%d",PLA_01.PLABrkEnable);
  // // ROS_INFO("BrkTrqAndDeceSwt:%d",PLA_01.BrkTrqAndDeceSwt);
  // // ROS_INFO("PLAParking:%d",PLA_01.PLAParking);
  // // ROS_INFO("PLAPrkDistance:%f",PLA_01.PLAPrkDistance);
  
  // // ROS_INFO("PLASignalTxCyclic:%d",PLA_01.PLASignalTxCyclic);
}
//==================================================================

//==================================================================
void canmessages::CANbusESP_02(stCANMsg *frame)
{
  ESP_02.YawState=0;
  ESP_02.YawRate=0; 
  ESP_02.YawToRight=0; 
  ESP_02.axState=0;  
  ESP_02.ax=0; 
  ESP_02.ayState=0;
  ESP_02.ay=0; 
  uint8 *pData = frame->data;
  int16 temp,temp1,temp2,temp3;

  temp =0;
  temp=(short)(pData[1]&0x40);
  temp=temp>>6;
  ESP_02.ayState=temp;

  temp =0;
  temp=(short)(pData[1]&0x20);
  temp=temp>>5;
  ESP_02.axState=temp;

  temp =0;
  temp=(short)(pData[1]&0x10);
  temp=temp>>4;
  ESP_02.YawState=temp;

  temp =0;
  temp=(short)(pData[2]&0xff);
  ESP_02.ay=temp*0.01-1.27;

  temp=0;
  temp1=0;
  temp=(short)(pData[3]&0xff);
  temp1=(short)(pData[4]&0x3);
  temp1=temp1<<8;
  temp1+=temp;
  ESP_02.ax=temp1*0.03125-16; 

  temp=0;
  temp1=0;
  temp=(short)(pData[6]&0xff);
  temp1=(short)(pData[7]&0x3F);
  temp1=temp1<<8;
  temp1+=temp;
  ESP_02.YawRate=temp1*0.01;

  temp =0;
  temp=(short)(pData[6]&0x40);
  temp=temp>>6;
  ESP_02.YawToRight=temp;

  // ROS_INFO("YawState:%d",ESP_02.YawState);
  // ROS_INFO("YawRate:%f",ESP_02.YawRate);
  // ROS_INFO("YawToRight:%d",ESP_02.YawToRight);
  // ROS_INFO("axState:%d",ESP_02.axState);
  // ROS_INFO("ax:%f", ESP_02.ax);
  // ROS_INFO("ayState:%d", ESP_02.ayState);
  // ROS_INFO("ay:%f",ESP_02.ay);
}
//===========================================
void canmessages::CANbusESP_05(stCANMsg *frame)
{
  ESP_05.BrakePressureState=0;
  ESP_05.BrakePressure=0; 
   uint8 *pData = frame->data;
   int16 temp,temp1,temp2,temp3;
   
   temp =0;
   temp=(short)(pData[1]&0x10);
   temp=temp>>4;
   ESP_05.BrakePressureState=temp;
   
   temp=0;
   temp1=0;
   temp=(short)(pData[2]&0xff);
   temp1=(short)(pData[3]&0x3F);
   temp1=temp1<<8;
   temp1+=temp;
   ESP_05.BrakePressure=temp1*0.3-30;
   
   // ROS_INFO("BrakePressureState:%d",ESP_05.BrakePressureState);
   // ROS_INFO("BrakePressure:%f",ESP_05.BrakePressure);
}
//===========================================
void canmessages::CANbusESP_10(stCANMsg *frame)
{
  ESP_10.LF_WhlSpdDrt=0;
  ESP_10.LR_WhlSpdDrt=0;
  ESP_10.RF_WhlSpdDrt=0;
  ESP_10.RR_WhlSpdDrt=0;  
  uint8 *pData = frame->data;
  int16 temp,temp1,temp2,temp3;

  temp =0;
  temp=(short)(pData[7]&0x3);
  ESP_10.LF_WhlSpdDrt=temp;

  temp =0;
  temp=(short)(pData[7]&0xC);
  temp=temp>>2;
  ESP_10.RF_WhlSpdDrt=temp;

  temp =0;
  temp=(short)(pData[7]&0x30);
  temp=temp>>4;
  ESP_10.LR_WhlSpdDrt=temp;

  temp =0;
  temp=(short)(pData[7]&0xC0);
  temp=temp>>6;
  ESP_10.RR_WhlSpdDrt=temp;

  // ROS_INFO("LF_WhlSpdDrt:%f",ESP_10.LF_WhlSpdDrt);
  // ROS_INFO("LR_WhlSpdDrt:%f",ESP_10.LR_WhlSpdDrt);
  // ROS_INFO("RF_WhlSpdDrt:%f",ESP_10.RF_WhlSpdDrt);
  // ROS_INFO("RR_WhlSpdDrt:%f",ESP_10.RR_WhlSpdDrt);
}
//==========================================
void canmessages::CANbusACC_02(stCANMsg *frame)
{
  ACC_02.ACC_02CRC=0;
  ACC_02.ACC_02BZ=0; 
  ACC_02.ExpectedSpeed=0; 
  ACC_02.SpacingFactor=0;  
  ACC_02.NoiseAlarm=0; 
  ACC_02.SetSpacing=0;
  ACC_02.ImageAlarm=0; 
  ACC_02.ACC02Status=0; 
  uint8 *pData = frame->data;
  int16 temp,temp1,temp2,temp3;

  temp =0;
  temp=(short)(pData[0]&0xff);
  ACC_02.ACC_02CRC=temp;

  temp =0;
  temp=(short)(pData[1]&0xf);
  ACC_02.ACC_02BZ=temp;

  temp=0;
  temp1=0;
  temp=(short)(pData[1]&0xf0);
  temp=temp>>4;
  temp1=(short)(pData[2]&0x3f);
  temp1=temp1<<4;
  temp1+=temp;
  ACC_02.ExpectedSpeed=temp1*0.32; 

  temp=0;
  temp1=0;
  temp=(short)(pData[3]&0xff);
  temp1=(short)(pData[4]&0x3);
  temp1=temp1<<8;
  temp1+=temp;
  ACC_02.SpacingFactor=temp1*0.32;

  temp =0;
  temp=(short)(pData[4]&0xc);
  temp=temp>>2;
  ACC_02.NoiseAlarm=temp;

  temp =0;
  temp=(short)(pData[4]&0xE0);
  temp=temp>>5;
  ACC_02.SetSpacing=temp;

  temp =0;
  temp=(short)(pData[5]&0x1);
  ACC_02.ImageAlarm=temp;

  temp =0;
  temp=(short)(pData[7]&0xE0);
  temp=temp>>5;
  ACC_02.ACC02Status=temp;   

  // ROS_INFO("ACC_02CRC:%f",ACC_02.ACC_02CRC);
  // ROS_INFO("ACC_02BZ:%f",ACC_02.ACC_02BZ);
  // ROS_INFO("ExpectedSpeed:%f",ACC_02.ExpectedSpeed);
  // ROS_INFO("SpacingFactor:%f",ACC_02.SpacingFactor);
  // ROS_INFO("NoiseAlarm:%f", ACC_02.NoiseAlarm);
  // ROS_INFO("SetSpacing:%f", ACC_02.SetSpacing);
  // ROS_INFO("ImageAlarm:%d",ACC_02.ImageAlarm);
  // ROS_INFO("ACC02Status:%f",ACC_02.ACC02Status);
}

//==========================================
void canmessages::CANbusACC_06(stCANMsg *frame)
{
  ACC_06.ACC_06CRC=0;
  ACC_06.ACC_06BZ=0; 
  ACC_06.aLowerDeviation=0; 
  ACC_06.aExpectedValue=0;  
  ACC_06.aUpperDeviation=0; 
  ACC_06.DclrtGradRmdValue=0;
  ACC_06.AclrtGradRmdValue=0; 
  ACC_06.StartRequest06=0; 
  ACC_06.ParkingRequest06=0; 
  ACC_06.ACC06Status=0; 
  uint8 *pData = frame->data;
  int16 temp,temp1,temp2,temp3;

  temp =0;
  temp=(short)(pData[0]&0xff);
  ACC_06.ACC_06CRC=temp;

  temp =0;
  temp=(short)(pData[1]&0xf);
  ACC_06.ACC_06BZ=temp;

  temp =0;
  temp=(short)(pData[2]&0x3f);
  ACC_06.aLowerDeviation=temp*0.024;

  temp=0;
  temp1=0;
  temp=(short)(pData[3]&0xff);
  temp1=(short)(pData[2]&0x7);
  temp1=temp1<<8;
  temp1+=temp;
  ACC_06.aExpectedValue=temp1*0.005-7.22; 

  temp =0;
  temp=(short)(pData[4]&0xF8);
  temp=temp>>3;
  ACC_06.aUpperDeviation=temp*0.0625;   

  temp =0;
  temp=(short)(pData[5]&0xff);
  ACC_06.DclrtGradRmdValue=temp*0.05;

  temp =0;
  temp=(short)(pData[6]&0xff);
  ACC_06.AclrtGradRmdValue=temp*0.05;

  temp =0;
  temp=(short)(pData[7]&0x1);
  ACC_06.StartRequest06=temp;

  temp =0;
  temp=(short)(pData[7]&0x2);
  temp=temp>>1;
  ACC_06.ParkingRequest06=temp;

  temp =0;
  temp=(short)(pData[4]&0x70);
  temp=temp>>4;
  ACC_06.ACC06Status=temp;

  // ROS_INFO("ACC_06CRC:%f",ACC_06.ACC_06CRC);
  // ROS_INFO("ACC_06BZ:%f",ACC_06.ACC_06BZ);
  // ROS_INFO("aLowerDeviation:%f",ACC_06.aLowerDeviation);
  // ROS_INFO("aExpectedValue:%f",ACC_06.aExpectedValue);
  // ROS_INFO("aUpperDeviation:%f", ACC_06.aUpperDeviation);
  // ROS_INFO("DclrtGradRmdValue:%f", ACC_06.DclrtGradRmdValue);
  // ROS_INFO("AclrtGradRmdValue:%f",ACC_06.AclrtGradRmdValue);
  // ROS_INFO("StartRequest06:%d",ACC_06.StartRequest06);
  // ROS_INFO("ParkingRequest06:%d",ACC_06.ParkingRequest06);
  // ROS_INFO("ACC06Status:%f",ACC_06.ACC06Status);
}

//==========================================
void canmessages::CANbusACC_07(stCANMsg *frame)
{
  ACC_07.ACC_07CRC=0;
  ACC_07.ACC_07BZ=0; 
  ACC_07.ParkingDistance=0; 
  ACC_07.ParkingRequest07=0;  
  ACC_07.StartRequest07=0; 
  ACC_07.aExpValue=0;
  uint8 *pData = frame->data;
  int16 temp,temp1,temp2,temp3;

  temp =0;
  temp=(short)(pData[0]&0xff);
  ACC_07.ACC_07CRC=temp;

  temp =0;
  temp=(short)(pData[1]&0xf);
  ACC_07.ACC_07BZ=temp;

  temp=0;
  temp1=0;
  temp=(short)(pData[1]&0xf0);
  temp=temp>>4;
  temp1=(short)(pData[2]&0x7F);
  temp1=temp1<<4;
  temp1+=temp;
  ACC_07.ParkingDistance=temp1*0.01; 

  temp =0;
  temp=(short)(pData[2]&0x80);
  temp=temp>>7;
  ACC_07.ParkingRequest07=temp;

  temp =0;
  temp=(short)(pData[3]&0x80);
  temp=temp>>7;
  ACC_07.StartRequest07=temp;   

  temp=0;
  temp1=0;
  temp=(short)(pData[6]&0xE0);
  temp=temp>>5;
  temp1=(short)(pData[7]&0xFF);
  temp1=temp1<<3;
  temp1+=temp;
  ACC_07.aExpValue=temp1*0.005-7.22;


  // ROS_INFO("ACC_07CRC:%f",ACC_07.ACC_07CRC);
  // ROS_INFO("ACC_07CRC:%f",ACC_07.ACC_07BZ);
  // ROS_INFO("ParkingDistance:%f",ACC_07.ParkingDistance);
  // ROS_INFO("ParkingRequest07:%d",ACC_07.ParkingRequest07);
  // ROS_INFO("StartRequest07:%d", ACC_07.StartRequest07);
  // ROS_INFO("aExpValue:%d", ACC_07.aExpValue);
}
//==========================================

//==========================================
void canmessages::pubLrrRadarData() 
{ 
  tourancan::can canmsg;

	canmsg.LWI_StrWhlAngleSt = LWI_01.LWI_StrWhlAngleSt;
	canmsg.LWI_StrWhlAngleSt = LWI_01.LWI_StrWhlAngleSt;
	canmsg.LWI_StrWhlAngleDrt = LWI_01.LWI_StrWhlAngleDrt;
	canmsg.LWI_StrWhlSpeedDrt =  LWI_01.LWI_StrWhlSpeedDrt; 
	canmsg.LWI_StrWhlSpeedSize = LWI_01.LWI_StrWhlSpeedSize;

	canmsg.TargetStalls = Getriebe_11.TargetStalls;
	canmsg.Stalls = Getriebe_11.Stalls;

	canmsg.EngineSpeedState = Gateway_73.EngineSpeedState;
	canmsg.EngineSpeed = Gateway_73.EngineSpeed;

	canmsg.ACCSignalContinuity = ESP_33.ACCSignalContinuity;

	canmsg.Speed = ESP_21.Speed;
	canmsg.ESP_SystemStatus = ESP_21.ESP_SystemStatus;
	canmsg.SpeedState = ESP_21.SpeedState;

	canmsg.LF_WhlSpd = ESP_19.LF_WhlSpd;
	canmsg.LR_WhlSpd = ESP_19.LR_WhlSpd;
	canmsg.RF_WhlSpd = ESP_19.RF_WhlSpd;
	canmsg.RR_WhlSpd = ESP_19.RR_WhlSpd;

	canmsg.aPedalPercent = Motor_20.aPedalPercent;
	canmsg.aPedalPercentSt = Motor_20.aPedalPercentSt;
	canmsg.ThrottleGradientSize = Motor_20.ThrottleGradientSize;
	canmsg.EngNeutralTorque = Motor_20.EngNeutralTorque;

	canmsg.BrakeSwitch = Motor_14.BrakeSwitch;

	canmsg.EPSRxHCA_Status = LH_EPS_03.EPSRxHCA_Status;
	canmsg.EPS_StrWhlTorque = LH_EPS_03.EPS_StrWhlTorque;
	canmsg.EPS_StrWhlTorqueDrt = LH_EPS_03.EPS_StrWhlTorqueDrt;
	canmsg.EPS_StrWhlTorqueSt = LH_EPS_03.EPS_StrWhlTorqueSt;

	canmsg.ExpStrWhlTorque = HCA_01.ExpStrWhlTorque;
	canmsg.HCATranCycle = HCA_01.HCATranCycle;
	canmsg.ExpStrWhlTorqueDrt = HCA_01.ExpStrWhlTorqueDrt;
	canmsg.HCA_Status = HCA_01.HCA_Status;

	canmsg.EngTorqueCoefficient = Motor_Code_01.EngTorqueCoefficient;

	canmsg.ThtottlePosition = OBD_01.ThtottlePosition;
	canmsg.aPedalPosition = OBD_01.aPedalPosition;

	canmsg.PLA_CRC = PLA_01.PLA_CRC;
	canmsg.PLA_BZ = PLA_01.PLA_BZ;
	canmsg.PLABrkRqtSt = PLA_01.PLABrkRqtSt;
	canmsg.PLAExpStrWhlAngle = PLA_01.PLAExpStrWhlAngle;
	canmsg.PLAExpStrWhlAngleDrt = PLA_01.PLAExpStrWhlAngleDrt;
	canmsg.PLARequestStatus = PLA_01.PLARequestStatus;
	canmsg.PLABrkTorque = PLA_01.PLABrkTorque;
	canmsg.PLABrkDeceleration = PLA_01.PLABrkDeceleration;
	canmsg.PLABrkEnable = PLA_01.PLABrkEnable;
	canmsg.BrkTrqAndDeceSwt = PLA_01.BrkTrqAndDeceSwt;
	canmsg.BrkTrqAndDeceSwt = PLA_01.PLAParking;
	canmsg.PLAPrkDistance = PLA_01.PLAPrkDistance;
	canmsg.PLASignalTxCyclic = PLA_01.PLASignalTxCyclic;

	canmsg.YawState = ESP_02.YawState;
	canmsg.YawRate = ESP_02.YawRate;
	canmsg.YawToRight = ESP_02.YawToRight;
	canmsg.axState = ESP_02.axState;
	canmsg.ax = ESP_02.ax;
	canmsg.ayState = ESP_02.ayState;
	canmsg.ay = ESP_02.ay;

	canmsg.BrakePressureState = ESP_05.BrakePressureState;
	canmsg.BrakePressure = ESP_05.BrakePressure;

	canmsg.LF_WhlSpdDrt = ESP_10.LF_WhlSpdDrt;
	canmsg.LR_WhlSpdDrt = ESP_10.LR_WhlSpdDrt;
	canmsg.RF_WhlSpdDrt = ESP_10.RF_WhlSpdDrt;
	canmsg.RR_WhlSpdDrt = ESP_10.RR_WhlSpdDrt;

	canmsg.ACC_02CRC = ACC_02.ACC_02CRC;
	canmsg.ACC_02BZ = ACC_02.ACC_02BZ;
	canmsg.ExpectedSpeed = ACC_02.ExpectedSpeed;
	canmsg.SpacingFactor = ACC_02.SpacingFactor;
	canmsg.NoiseAlarm = ACC_02.NoiseAlarm;
	canmsg.SetSpacing = ACC_02.SetSpacing;
	canmsg.ImageAlarm = ACC_02.ImageAlarm;
	canmsg.ACC02Status = ACC_02.ACC02Status;

	canmsg.ACC_06CRC = ACC_06.ACC_06CRC;
	canmsg.ACC_06BZ = ACC_06.ACC_06BZ;
	canmsg.aLowerDeviation = ACC_06.aLowerDeviation;
	canmsg.aExpectedValue = ACC_06.aExpectedValue;
	canmsg.aUpperDeviation = ACC_06.aUpperDeviation;
	canmsg.DclrtGradRmdValue = ACC_06.DclrtGradRmdValue;
	canmsg.AclrtGradRmdValue = ACC_06.AclrtGradRmdValue;
	canmsg.StartRequest06 = ACC_06.StartRequest06;
	canmsg.ParkingRequest06 = ACC_06.ParkingRequest06;
	canmsg.ACC06Status = ACC_06.ACC06Status;

	canmsg.ACC_07CRC = ACC_07.ACC_07CRC;
	canmsg.ACC_07BZ = ACC_07.ACC_07BZ;
	canmsg.ParkingDistance = ACC_07.ParkingDistance;
	canmsg.ParkingRequest07 = ACC_07.ParkingRequest07;
	canmsg.StartRequest07 = ACC_07.StartRequest07;
	canmsg.aExpValue = ACC_07.aExpValue;

	canmsg.TSK_status = TSK_06.TSK_status;

	canmsg.EPBFaultStatus = EPB_01.EPBFaultStatus;
	canmsg.EPBSwitch = EPB_01.EPBSwitch;
	canmsg.EPBSwitchState = EPB_01.EPBSwitchState;
	canmsg.EPB_PressingForce = EPB_01.EPB_PressingForce;
	canmsg.EPB_Status = EPB_01.EPB_Status;

  pub_can.publish(canmsg); // publish msg
  //loop_rate.sleep();
}


       


