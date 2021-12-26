#ifndef __Can_SCU_IPC_1_0x174_H__
#define __Can_SCU_IPC_1_0x174_H__

#include "structureCanP7.h"

class Can_SCU_IPC_1_0x174
{
public:
	Can_SCU_IPC_1_0x174(){};
	~Can_SCU_IPC_1_0x174(){};
	void decode_174(uint8 *msg_)
	{
		data_.SteeringAngleVD = 0;
		data_.SteeringAngleSpd = 0;
		data_.SteeringAngle = -780;
		data_.ResponseTorque = 0;
		data_.ResponseTorqueVD = 0;
		data_.SteeringAngleSpdVD = 0;
		data_.MsgCounter = 0;
		data_.Checksum = 0;

		uint8 *pData = msg_;
		int16 temp, temp1, temp2, temp3;
		// cout << "0x174: " << endl;

		temp = 0;
		temp = (short)((pData[0] >> 7) & 0x01);
		data_.SteeringAngleVD = temp;
		//cout << "Steering angle VD = " << (uint16)data_.SteeringAngleVD << endl;

		temp = 0;
		temp1 = 0;
		temp = (short)(pData[0] & 0x7f);
		temp1 = (short)((pData[1] >> 5) & 0x07);
		temp = temp << 3;
		data_.SteeringAngleSpd = temp + temp1;
		//cout << "Steering angle speed = " << data_.SteeringAngleSpd << " deg/s "<< endl;

		temp = 0;
		temp1 = 0;
		temp2 = 0;
		temp = (short)(pData[1] & 0x1f);
		temp1 = (short)(pData[2] & 0xff);
		temp2 = (short)((pData[3] >> 5) & 0x07);
		temp = temp << 11;
		temp1 = temp1 << 3;
		temp2 = temp2;
		data_.SteeringAngle = temp + temp1 + temp2;
		//cout << "Steering angle = " << data_.SteeringAngle / 10 - 780 << " deg " << endl;

		temp = 0;
		temp1 = 0;
		temp = (short)(pData[3] & 0x1f);
		temp1 = (short)((pData[4] >> 2) & 0x3f);
		temp = temp << 6;
		data_.ResponseTorque =temp + temp1;
		//cout << "Response torque = " << data_.ResponseTorque / 100 - 10.24 << "N·m" << endl;

		temp = 0;
		temp = (short)((pData[4] >> 1) & 0x01);
		data_.ResponseTorqueVD = temp;
		//cout << "Response torque VD = " << (uint16)data_.ResponseTorqueVD << endl;

		temp = 0;
		temp = (short)(pData[4] & 0x01);
		data_.SteeringAngleSpdVD = temp;
		//cout << "Steering angle speed VD = " << (uint16)data_.SteeringAngleSpdVD << endl;

		temp = 0;
		temp = (short)(pData[6] & 0x0f);
		data_.MsgCounter = temp;
		//cout << "Message counter 174 =" << (uint16)data_.MsgCounter << endl;

		temp = 0;
		temp = (short)(pData[7] & 0xff);
		data_.Checksum = temp;
		//cout << "checksum 174 = " << (uint16)data_.Checksum << endl;
	};													//end of decode()
	CanFrame_SCU_IPC_1_0x174 *data() { return &data_; } //把解析结果引出来
protected:
private:
	CanFrame_SCU_IPC_1_0x174 data_;
}; //end of class  Can_SCU_IPC_1_0x174

#endif //end of __Can_SCU_IPC_1_0x174_H__
