#ifndef __Can_SCU_IPC_4_0x177_H__
#define __Can_SCU_IPC_4_0x177_H__

#include "structureCanP7.h"

class Can_SCU_IPC_4_0x177
{
public:
	Can_SCU_IPC_4_0x177(){};
	~Can_SCU_IPC_4_0x177(){};
	void decode_177(uint8 *msg_)
	{
		data_.ActVehLongAccelVD = 0;
		data_.ActVehLateralAccelVD = 0;
		data_.YAWVD = 0;
		data_.YAW = 0;
		data_.ActVehLongAccel = 0;
		data_.ActVehLateralAccel = 0;
		data_.EPBSysSt = 0;
		data_.MsgCounter = 0;
		data_.Checksum = 0;

		uint8 *pData = msg_;
		int16 temp, temp1, temp2, temp3;

		// cout<< "0x177: " << endl;
		temp = 0;
		temp = (short)((pData[0] >> 7) & 0x01);
		data_.ActVehLongAccelVD = temp;
		//cout<< "Act Vehicle longitudinal Acceleration VD = " << (uint16)data_.ActVehLateralAccelVD << endl;

		temp = 0;
		temp = (short)((pData[0] >> 6) & 0x01);
		data_.ActVehLateralAccelVD = temp;
		//cout<< "Act Vehicle lateral acceleration VD = " << (uint16)data_.ActVehLateralAccelVD << endl;

		temp = 0;
		temp = (short)((pData[0] >> 5) & 0x01);
		data_.YAWVD = temp;
		//cout<< "Yaw VD = " << (uint16)data_.YAWVD << endl;

		temp = 0;
		temp1 = 0;
		temp = (short)(pData[0] & 0x1f);
		temp1 = (short)((pData[1] >> 1) & 0x7f);
		temp = temp << 7;
		data_.YAW = temp + temp1;
		//cout<< "Yaw = " << data_.YAW * 0.0625 - 93 << " deg/s "<< endl;

		temp = 0;
		temp1 = 0;
		temp2 = 0;
		temp = (short)(pData[1] & 0x01);
		temp1 = (short)(pData[2] & 0xff);
		temp2 = (short)((pData[3] >> 5) & 0x07);
		temp = temp << 11;
		temp1 = temp1 << 3;
		temp2 = temp2;
		data_.ActVehLongAccel = temp + temp1 + temp2;
		cout<< "Act vehicle longitudinal acceleration = " << (data_.ActVehLongAccel * 0.002768 - 1.8)*9.8 << " m/s^2 " << endl;

		temp = 0;
		temp1 = 0;
		temp = (short)(pData[3] & 0x1f);
		temp1 = (short)((pData[4] >> 1) & 0x7f);
		temp = temp << 7;
		data_.ActVehLateralAccel = temp + temp1;
		//cout<< "Act vehicle lateral acceleration = " << data_.ActVehLateralAccel / 0.002768 - 1.8 << " g " << endl;

		temp = 0;
		temp1 = 0;
		temp = (short)(pData[4] & 0x01);
		temp1 = (short)((pData[5] >> 6) & 0x03);
		temp = temp << 2;
		data_.EPBSysSt = temp + temp1;
		//cout<< "EPB system status = " << (uint16)data_.EPBSysSt << endl;

		temp = 0;
		temp = (short)(pData[6] & 0x0f);
		data_.MsgCounter = temp;
		//cout<< "Message counter 177 = " << (uint16)data_.MsgCounter << endl;

		temp = 0;
		temp = (short)(pData[7] & 0xff);
		data_.Checksum = temp;
		//cout<< "Checksum 177 = " << (uint16)data_.Checksum << endl;
	}; //end of decode()
	CanFrame_SCU_IPC_4_0x177 *data() { return &data_; };

protected:
private:
	CanFrame_SCU_IPC_4_0x177 data_;
}; //end of class  Can_SCU_IPC_4_0x177

#endif //end of __Can_SCU_IPC_4_0x177_H__
