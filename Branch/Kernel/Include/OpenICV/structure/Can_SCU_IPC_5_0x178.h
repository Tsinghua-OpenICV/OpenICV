#ifndef __Can_SCU_IPC_5_0x178_H__
#define __Can_SCU_IPC_5_0x178_H__

#include "structureCanP7.h"

class Can_SCU_IPC_5_0x178
{
public:
	Can_SCU_IPC_5_0x178(){};
	~Can_SCU_IPC_5_0x178(){};
	void decode_178(uint8 *msg_)
	{
		data_.DriverDoorLockSt = 0;
		data_.DriverDoorAjarSt = 0;
		data_.PsngrDoorAjarSt = 0;
		data_.RLDoorAjarSt = 0;
		data_.RRDoorAjarSt = 0;
		data_.LTurnLampOutputSt = 0;
		data_.RTurnLampOutputSt = 0;
		data_.HazardLampOutputSt = 0;
		data_.LowBeamOutputSt = 0;
		data_.HighBeamOutputSt = 0;
		data_.Horndriverst = 0;
		data_.FrontWiperOutputSt = 1;
		data_.PowerMode = 0;

		uint8 *pData = msg_;
		int16 temp, temp1, temp2, temp3;

		// cout<< "0x178: " << endl;
		temp = 0;
		temp = (short)(pData[0] & 0x01);
		data_.DriverDoorLockSt = temp;
		//cout<< "Driver door lock status = " << (uint16)data_.DriverDoorLockSt << endl;

		temp = 0;
		temp = (short)((pData[0] >> 1) & 0x01);
		data_.DriverDoorAjarSt = temp;
		//cout<< "Driver door ajar status = " << (uint16)data_.DriverDoorAjarSt << endl;

		temp = 0;
		temp = (short)((pData[0] >> 2) & 0x01);
		data_.PsngrDoorAjarSt = temp;
		//cout<< "Passenger door ajar status = " << (uint16)data_.PsngrDoorAjarSt << endl;

		temp = 0;
		temp = (short)((pData[0] >> 3) & 0x01);
		data_.RLDoorAjarSt = temp;
		//cout<< "Rear left door ajar status = " << (uint16)data_.RLDoorAjarSt << endl;

		temp = 0;
		temp = (short)((pData[0] >> 4) & 0x01);
		data_.RRDoorAjarSt = temp;
		//cout<< "Rear right door ajar status = " << (uint16)data_.RRDoorAjarSt << endl;

		temp = 0;
		temp = (short)((pData[0] >> 5) & 0x01);
		data_.LTurnLampOutputSt = temp;
		//cout<< "Left turn lamp output status = " << (uint16)data_.LTurnLampOutputSt << endl;

		temp = 0;
		temp = (short)((pData[0] >> 6) & 0x01);
		data_.RTurnLampOutputSt = temp;
		//cout<< "Right turn lamp output status = " << (uint16)data_.RTurnLampOutputSt << endl;

		temp = 0;
		temp = (short)((pData[0] >> 7) & 0x01);
		data_.HazardLampOutputSt = temp;
		//cout<< "Hazard lamp output status = " << (uint16)data_.HazardLampOutputSt << endl;

		temp = 0;
		temp = (short)((pData[1] >> 7) & 0x01);
		data_.LowBeamOutputSt = temp;
		//cout<< "Low beam output status = " << (uint16)data_.LowBeamOutputSt << endl;

		temp = 0;
		temp = (short)((pData[1] >> 6) & 0x01);
		data_.HighBeamOutputSt = temp;
		//cout<< "High beam output status = " << (uint16)data_.HighBeamOutputSt << endl;

		temp = 0;
		temp = (short)((pData[1] >> 5) & 0x01);
		data_.Horndriverst = temp;
		//cout<< "Horn driver status = " << (uint16)data_.Horndriverst << endl;

		temp = 0;
		temp = (short)((pData[1] >> 3) & 0x03);
		data_.FrontWiperOutputSt = temp;
		//cout<< "Front wiper output status = " << (uint16)data_.FrontWiperOutputSt << endl;

		temp = 0;
		temp = (short)((pData[1] >> 1) & 0x03);
		data_.PowerMode = temp;
		//cout<< "Power mode = " << (uint16)data_.PowerMode << endl;
	}; //end of decode()
	CanFrame_SCU_IPC_5_0x178 *data() { return &data_; };

protected:
private:
	CanFrame_SCU_IPC_5_0x178 data_;
}; //end of class  Can_SCU_IPC_5_0x178

#endif //end of __Can_SCU_IPC_5_0x178_H__
