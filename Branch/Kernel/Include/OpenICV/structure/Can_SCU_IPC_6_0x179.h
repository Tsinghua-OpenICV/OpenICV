#ifndef __Can_SCU_IPC_6_0x179_H__
#define __Can_SCU_IPC_6_0x179_H__

#include "structureCanP7.h"

class Can_SCU_IPC_6_0x179
{
public:
	Can_SCU_IPC_6_0x179(){};
	~Can_SCU_IPC_6_0x179(){};
	void decode_179(uint8 *msg_)
	{
		data_.dstBat_Dsp = 0;
		data_.AccPedalSig = 0;
		data_.BrkPedalSt = 0;
		data_.BrkPedalStVD = 0;
		data_.CurrentGearLevVD = 0;
		data_.CurrentGearLev = 0;
		data_.MsgCounter = 0;
		data_.Checksum = 0;

		uint8 *pData = msg_; //pData就是recv_msg[i].data
		int16 temp, temp1, temp2, temp3;

		// cout<< "0x179: " << endl;

		temp = 0;
		temp1 = 0;
		temp = (short)(pData[0] & 0xff);
		temp1 = (short)((pData[1] >> 6) & 0x03);
		temp = temp << 2;
		data_.dstBat_Dsp = temp + temp1;
		//cout<< "distance battery display = " << data_.dstBat_Dsp << endl;
		
		temp = 0;
		temp1 = 0;
		temp = (short)(pData[1] & 0x3f);
		temp1 = (short)((pData[2] >> 6) & 0x03);
		temp = temp << 2;
		data_.AccPedalSig = temp + temp1;
		//cout<< "Acceleration pedal signal = " << data_.AccPedalSig << endl;

		temp = 0;
		temp = (short)((pData[2] >> 5) & 0x01);
		data_.BrkPedalSt = temp;
		//cout<< "Break pedal status = " << (uint16)data_.BrkPedalSt << endl;

		temp = 0;
		temp = (short)((pData[2] >> 4) & 0x01);
		data_.BrkPedalStVD = temp;
		//cout<< "Break pedal status VD = " << (uint16)data_.BrkPedalStVD << endl;

		temp = 0;
		temp = (short)((pData[2] >> 3) & 0x01);
		data_.CurrentGearLevVD = temp;
		//cout<< "Current gear level VD = " << (uint16)data_.CurrentGearLevVD << endl;

		temp = 0;
		temp = (short)(pData[2] & 0x07);
		data_.CurrentGearLev = temp;
		//cout<< "Current gear level = " << (uint16)data_.CurrentGearLev << endl;

		temp = 0;
		temp = (short)(pData[6] & 0x0f);
		data_.MsgCounter = temp;
		//cout<< "Message counter 179 = " << (uint16)data_.MsgCounter << endl;

		temp = 0;
		temp = (short)(pData[7] & 0xff);
		data_.Checksum = temp;
		//cout<< "Checksum 179 = " << (uint16)data_.Checksum << endl;
	}; //end of decode()
	CanFrame_SCU_IPC_6_0x179 *data() { return &data_; };

protected:
private:
	CanFrame_SCU_IPC_6_0x179 data_;
}; //end of class  Can_SCU_IPC_6_0x179

#endif //end of __Can_SCU_IPC_6_0x179_H__
