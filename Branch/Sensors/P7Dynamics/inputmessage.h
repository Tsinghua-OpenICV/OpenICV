//#ifndef __inputmessage_h__
//#define _inputmessage_h__

#ifndef __INPUTMESSAGE_H__
#define __INPUTMESSAGE_H__

#include <stdint.h>
//#include "structureCanXP.h"

typedef struct
{
	uint8_t IPC_SCU_DBWReq;
	uint8_t IPC_SCU_ReadyReq;
	uint8_t IPC_SCU_BrakeLight;
	uint16_t IPC_SCU_SteerAngleReq;
	uint8_t IPC_SCU_SteerAngleReqVD;
	uint16_t IPC_SCU_TorsionBarTqReq;
	uint8_t IPC_SCU_TorsionBarTqReqVD;
	uint8_t IPC_SCU_GearReq;
	uint8_t IPC_SCU_1_MsgCounter;
	uint8_t IPC_SCU_1_Checksum;
} inputmessage_171;

typedef struct
{
	uint16_t IPC_SCU_MotorTorqReq;
	uint8_t IPC_SCU_MotorTorqReqVD;
	uint8_t IPC_SCU_ParkingReqToEPB;
	uint8_t IPC_SCU_ParkingReqToEPBVD;
	uint16_t IPC_SCU_AccDecelReq;
	uint8_t IPC_SCU_AccDecelReqVD;
	uint16_t IPC_SCU_VehSpd;
	uint8_t IPC_SCU_VehSpdVD;
	uint8_t IPC_SCU_2_MsgCounter;
	uint8_t IPC_SCU_2_Checksum;
} inputmessage_172;

typedef struct
{
	uint8_t IPC_SCU_LowBeamSWSt;
	uint8_t IPC_SCU_LowBeamSWStVD;
	uint8_t IPC_SCU_HighBeamSWSt;
	uint8_t IPC_SCU_HighBeamSWStVD;
	uint8_t IPC_SCU_HazardLampSWSt;
	uint8_t IPC_SCU_LTurnLampSWSt;
	uint8_t IPC_SCU_RTurnLampSWSt;
	uint8_t IPC_SCU_HorndriverSWst;
} inputmessage_173;

#endif
