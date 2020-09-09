#ifndef _AccContorl_H
#define _AccContorl_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Basis/icvStructureData.hxx"

//#include "OpenICV/Function/icvUdpReceiverSource.h"
#include "OpenICV/structure/structureCanTuan.h"
#include "OpenICV/structure/structureVehinfo.h"
#include "OpenICV/structure/structure_gps.h"
using namespace icv;
using namespace core;

class ACCcontrol: public icvFunction
{
public:
  typedef data::icvStructureData<double>    icvDouble;

  ACCcontrol(icv_shared_ptr<const icvMetaData> info) : icvFunction(info){
	  Register_Sub("acc_off");
	  Register_Sub("Veh_info");
	    };
//   virtual void ConfigurateOutput(std::vector<icvPublisher*>& outputPorts) override {
// 	  			ini_Output(outputPorts,1);
// icvDataObject* outdatai=new icvDouble;
// outdatai->Reserve();
//   outputPorts[0]->SetDataObject(outdatai);

//   }
  
//     virtual void ConfigurateInput(std::vector<icvSubscriber*>& inputPorts) override
//     {
// 			ini_Input(inputPorts,2);

//           icvPublisher* output_con1= inputPorts[0]->GetConnections()[0];
// 	  bool correct;
// 	   if(output_con1->RequireDataObject(false))
//              correct= dynamic_cast<data::icvInt32Data*>(output_con1->RequireDataObject(false));
//         else
//         {
//             output_con1->SetDataObject((icvDataObject*)(new data::icvInt32Data()));
//             correct= true;
//         }
// 		if (!correct)ICV_LOG_TRACE<<"Data type not matched";
// 		else ICV_LOG_TRACE<<"Data type  matched";

//       icvPublisher* output_con= inputPorts[1]->GetConnections()[0];
// 	   correct;
// 	   if(output_con->RequireDataObject(false))
//              correct= dynamic_cast<data::icvStructureData<veh_info>*>(output_con->RequireDataObject(false));
//         else
//         {
			
//             output_con->SetDataObject((icvDataObject*)(new data::icvStructureData<veh_info>()));
//             correct= true;
//         }
// 		if (!correct)ICV_LOG_TRACE<<"Data type not matched";
// 		else ICV_LOG_TRACE<<"Data type  matched";

// }
void chatcallback(const veh_info& canmsg)//回调函数  
{  
   // ROS_INFO("I heared Speed is:[%f]",canmsg->Speed); 
   // ROS_INFO("I heared ax is:[%f]",canmsg->ax);
    ACC_v=canmsg.Speed	;
    ACC_ax=canmsg.ax;
}

void accoffcallback(const veh_info& accoffmsg)//回调函数 
{
    acc_off=accoffmsg.data;
}

    virtual void Execute() override
    {
        data::icvInt64Data acc_off;
		icvSubscribe("acc_off",&acc_off);
		
		data::icvStructureData<veh_info> veh_infomation;
        icvSubscribe("veh_info",&veh_infomation);

		chatcallback(veh_infomation);
		count_++ ;
    float V_s;          //costant number,known,
    float V_set;        //costant number,known
    float a_accel_max;  //costant number,known
    float a_decel_desired;  //costant number,known
    float constant;   //costant number,known
    float delta_T;    //costant number,known,related to the number of rate
    float K_SVE;     //costant number,known
    float d_SVE;     //costant number,known
    float k_v_set;

    //V_s=ACC_v;
    //V_s=V_s/3.6;
    V_set=7.2;      //set the cruise speed
    V_set=V_set/3.6;
    a_accel_max=2.0;   //postive number
    a_decel_desired=5.0;  //neagtive number
    constant=3.0;  //the change rate of a
    delta_T=1.0/5.0;  //1/10,related to rate number
    //delta_T=0.01;  //1/10,related to rate number
    K_SVE=-0.002;
    d_SVE=1.025; 
    k_v_set=0.5583;
    

    float inv_SVE;  //unknown
    float SVE; //unknown
    float accel;  //unknown
    float accel_pre;  //unknown,storage last moment accel


				//mFrame.copyTo(frame_send);
			//ICV_LOG_TRACE<<"ms: "<<icv::icvTime::now_ms().time_since_epoch().count()<<endl;
		//	ICV_LOG_TRACE<<"s"<<icv::icvTime::now_s().time_since_epoch().count()<<endl;

    V_s=ACC_v;
    V_s=V_s/3.6;
	accel_pre=ACC_ax;
    inv_SVE=K_SVE*V_s+d_SVE;
  	SVE=1/inv_SVE;
	accel=SVE*k_v_set*(V_set-V_s); //ideal accel
	if (count_==0)  //the first beginning part
	{
		if (accel>accel_pre)  //the speed is increase
		{
			if ((accel-accel_pre)/delta_T>constant)  accel=accel_pre+delta_T*constant;
			else  accel=accel;
			if  (accel>a_accel_max)  accel=a_accel_max;
			else  accel=accel;
		}
		else     //the speed is decrease
		{
			if (abs((accel-accel_pre)/delta_T)>constant)  accel=accel_pre+delta_T*(-constant);
			else  accel=accel;
			if  (accel<-a_decel_desired)  accel=-a_decel_desired;
			else  accel=accel;
		}
	 }
	else
	{ 
		if (accel>accel_pre)  //the speed is increase
		{
			if ((accel-accel_pre)/delta_T>constant)  accel=accel_pre+delta_T*constant;
			else  accel=accel;
			if  (accel>a_accel_max)  accel=a_accel_max;
			else  accel=accel;
		}	
		else  //the speed is decrease
		{
			if (abs((accel-accel_pre)/delta_T)>constant)  accel=accel_pre+delta_T*(-constant);
			else  accel=accel;
			if  (accel<-a_decel_desired)  accel=-a_decel_desired;
			else  accel=accel;
		}
	}

	if (acc_off>=5)   accel=-1.0;
    }
private:
float ACC_v; //the actual speed
float ACC_ax;//the actual accel
int count_=0;
int acc_off;

};
ICV_REGISTER_FUNCTION(ACCcontrol)

#endif

