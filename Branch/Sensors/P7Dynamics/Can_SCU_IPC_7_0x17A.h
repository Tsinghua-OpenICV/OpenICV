#ifndef __Can_SCU_IPC_7_0x17A_H__
#define __Can_SCU_IPC_7_0x17A_H__

#include "structureCanP7.h"

class Can_SCU_IPC_7_0x17A
{
public:
	Can_SCU_IPC_7_0x17A(){};
	~Can_SCU_IPC_7_0x17A(){};
	void decode_17A(uint8 *msg_)
	{
		data_.PSeatBeltWarning = 0;
		data_.DriverSeatBeltWarning = 0;
		data_._1ndLPSeatBeltWar = 0;
		data_._1ndMPSeatBeltWar = 0;
		data_._1ndRPSeatBeltWar = 0;
		data_.TotalOdometerVD = 0;
		data_.TotalOdometer = 0;
		data_.ACSt = 0;
		data_.ErrSt = 0;
		data_.OverrideRes = 0;

		uint8 *pData = msg_;
		int16 temp, temp1, temp2, temp3;

		// cout<< "0x17A: " << endl;

		temp = 0;
		temp = (short)((pData[0] >> 2) & 0x01);
		data_.PSeatBeltWarning = temp;
		//cout<< "Passenger seat belt warning = " << (uint16)data_.PSeatBeltWarning << endl;
		
		temp = 0;
		temp = (short)((pData[0] >> 3) & 0x01);
		data_.DriverSeatBeltWarning = temp;
		//cout<< "Driver seat belt warning = " << (uint16)data_.DriverSeatBeltWarning << endl;

		temp = 0;
		temp = (short)((pData[0] >> 4) & 0x01);
		data_._1ndLPSeatBeltWar = temp;
		//cout<< "Left passenger seat belt warning = " << (uint16)data_._1ndLPSeatBeltWar << endl;

		temp = 0;
		temp = (short)((pData[0] >> 5) & 0x01);
		data_._1ndMPSeatBeltWar = temp;
		//cout<< "Middle passenger seat belt warning = " << (uint16)data_._1ndMPSeatBeltWar << endl;

		temp = 0;
		temp = (short)((pData[0] >> 6) & 0x01);
		data_._1ndRPSeatBeltWar = temp;
		//cout<< "Right passenger seat belt warning = " << (uint16)data_._1ndRPSeatBeltWar << endl;

		temp = 0;
		temp = (short)((pData[0] >> 7) & 0x01);
		data_.TotalOdometerVD = temp;
		//cout<< "Total odometer VD = " << (uint16)data_.TotalOdometerVD << endl;

		temp = 0;
		temp1 = 0;
		temp2 = 0;
		temp3 = 0;
		temp = (short)(pData[0] & 0x03);
		temp1 = (short)(pData[1] & 0xff);
		temp2 = (short)(pData[2] & 0xff);
		temp3 = (short)((pData[3] >> 6) & 0x03);
		temp = temp << 18;
		temp1 = temp1 << 10;
		temp2 = temp2 << 2;
		data_.TotalOdometer = temp + temp1 + temp2 + temp3;
		//cout<< "Total odometer = " << data_.TotalOdometer << endl;

		temp = 0;
		temp = (short)((pData[3] >> 5) & 0x01);
		data_.ACSt = temp;
		//cout<< "Air conditioner status = " << (uint16)data_.ACSt << endl;

		temp = 0;
		temp = (short)((pData[3] >> 1) & 0x0f);
		data_.ErrSt = temp;
		//cout<< "Error status = " << (uint16)data_.ErrSt << endl;

		temp = 0;
		temp1 = 0;
		temp = (short)(pData[3] & 0x01);
		temp1 = (short)((pData[4] >> 5) & 0x07);
		temp = temp << 3;
		data_.OverrideRes = temp + temp1;
		//cout<< "Override res = " << (uint16)data_.OverrideRes << endl;
	}; //end of decode()
	CanFrame_SCU_IPC_7_0x17A *data() { return &data_; };

protected:
private:
	CanFrame_SCU_IPC_7_0x17A data_;
}; //end of class  Can_SCU_IPC_7_0x17A

#endif //end of __Can_SCU_IPC_7_0x17A_H__
