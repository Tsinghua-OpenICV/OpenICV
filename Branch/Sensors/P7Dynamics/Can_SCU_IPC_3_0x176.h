#ifndef __Can_SCU_IPC_3_0x176_H__
#define __Can_SCU_IPC_3_0x176_H__

#include "structureCanP7.h"

class Can_SCU_IPC_3_0x176
{
public:
	Can_SCU_IPC_3_0x176(){};
	~Can_SCU_IPC_3_0x176(){};
	void decode_176(uint8 *msg_)
	{
		data_.DBWSt = 0;
		data_.VehSpdVD = 0;
		data_.BrkPedalSt = 0;
		data_.BrkPedalStVD = 0;
		data_.VehSpd = 0;
		data_.BrkLightOn = 0;
		data_.MsgCounter = 0;
		data_.Checksum = 0;

		uint8 *pData = msg_;
		int16 temp, temp1, temp2, temp3;

		// cout<< "0x176: " << endl;

		temp = 0;
		temp = (short)((pData[0] >> 4) & 0x0f);
		data_.DBWSt = temp;
		//cout<< "Drive by wire status = " << (uint16)data_.DBWSt << endl;

		temp = 0;
		temp = (short)((pData[0] >> 3) & 0x01);
		data_.VehSpdVD = temp;
		//cout<< "Vehicle speed VD = " << (uint16)data_.VehSpdVD << endl;

		temp = 0;
		temp = (short)((pData[0] >> 2) & 0x01);
		data_.BrkPedalSt = temp;
		//cout<< "Break pedal status = " << (uint16)data_.BrkPedalSt << endl;

		temp = 0;
		temp = (short)((pData[0] >> 1) & 0x01);
		data_.BrkPedalStVD = temp;
		//cout<< "Break pedal status VD = " << (uint16)data_.BrkPedalStVD << endl;

		temp = 0;
		temp1 = 0;
		temp2 = 0;
		temp = (short)(pData[0] & 0x01);
		temp1 = (short)(pData[1] & 0xff);
		temp2 = (short)((pData[2] >> 4) & 0x0f);
		temp = temp << 12;
		temp1 = temp1 << 4;
		temp2 = temp2;
		data_.VehSpd = temp + temp1 + temp2;
		//cout<< "Vehicle speed = " << data_.VehSpd / 0.05625 << " km/h " << endl;

		temp = 0;
		temp = (short)((pData[2] >> 3) & 0x01);
		data_.BrkLightOn = temp;
		//cout<< "Break light on = " << (uint16)data_.BrkLightOn << endl;

		temp = 0;
		temp = (short)(pData[6] & 0x0f);
		data_.MsgCounter = temp;
		//cout<< "Message counter 176 = " << (uint16)data_.MsgCounter << endl;

		temp = 0;
		temp = (short)(pData[7] & 0xff);
		data_.Checksum = temp;
		//cout<< "Checksum 176 = " << (uint16)data_.Checksum << endl;
	}; //end of decode()
	CanFrame_SCU_IPC_3_0x176 *data() { return &data_; };

protected:
private:
	CanFrame_SCU_IPC_3_0x176 data_;
}; //end of class  Can_SCU_IPC_3_0x176

#endif //end of __Can_SCU_IPC_3_0x176_H__
