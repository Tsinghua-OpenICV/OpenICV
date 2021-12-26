
#ifndef _PyFrontend_H
#define _PyFrontend_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Core/icvTime.h"
#include "OpenICV/Core/icvConfig.h"
#include <Python.h>
#include "OpenICV/Basis/icvSemaphore.h"

#include "OpenICV/structure/icvCvMatData.hxx"
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/thread/thread.hpp>
#include <iostream>

#include <string>
#include <sstream>
#include <vector>
#include "data_demo_type1.h"

#include "OpenICV/structure/structureIMU.h"
#include "OpenICV/structure/structure_gps.h"
#include "OpenICV/Basis/icvStructureData.hxx"

#include "OpenICV/structure/Can_SCU_IPC_1_0x174.h"
#include "OpenICV/structure/Can_SCU_IPC_2_0x175.h"
#include "OpenICV/structure/Can_SCU_IPC_3_0x176.h"
#include "OpenICV/structure/Can_SCU_IPC_4_0x177.h"
#include "OpenICV/structure/Can_SCU_IPC_5_0x178.h"
#include "OpenICV/structure/Can_SCU_IPC_6_0x179.h"
#include "OpenICV/structure/Can_SCU_IPC_7_0x17A.h"
#include "OpenICV/structure/structureCanP7.h"
#include "OpenICV/structure/structure_all_objs.h"
#include <dlfcn.h>


#include <eigen3/Eigen/Dense>
#include "util.h"
#include "LocalCartesian.hpp"



void *handle =   dlopen("libpython2.7.so",RTLD_LAZY|RTLD_GLOBAL);
// ./icvStarter json_exec/PyFrontend.json

#define MAX_THREAD_COUNT 10
using namespace icv;
void signal_handler_fun(int signum)
{
    //if (signum == SIGINT) {

    printf("ctl C close the frontend\n");
    exit(0);
}   
typedef icv::data::icvStructureData<Imu>    icvImu;
typedef icv::data::icvStructureData<NavSatFix>    icvNavSatFix;
typedef icv::data::icvStructureData<TwistWithCovarianceStamped>    icvTwistWithCovarianceStamped;
typedef icv::data::icvStructureData<Odometry>    icvOdometry;

typedef data::icvStructureData<data_demo_type2> icvdata_demo_type2;

typedef icv::data::icvStructureData<CanFrame_SCU_IPC_1_0x174>  SCU_IPC_0x174;
typedef icv::data::icvStructureData<CanFrame_SCU_IPC_2_0x175>  SCU_IPC_0x175;
typedef icv::data::icvStructureData<CanFrame_SCU_IPC_3_0x176>  SCU_IPC_0x176;
typedef icv::data::icvStructureData<CanFrame_SCU_IPC_4_0x177>  SCU_IPC_0x177;
typedef icv::data::icvStructureData<CanFrame_SCU_IPC_5_0x178>  SCU_IPC_0x178;
typedef icv::data::icvStructureData<CanFrame_SCU_IPC_6_0x179>  SCU_IPC_0x179;
typedef icv::data::icvStructureData<CanFrame_SCU_IPC_7_0x17A>  SCU_IPC_0x17A;


class PyFrontend : public icvFunction
{
public:



    PyFrontend(icv_shared_ptr<const icvMetaData> info) : icvFunction(info)
    {
        ICV_LOG_INFO << "C++ Frontend of Python script Started";
        signal(SIGINT, signal_handler_fun); //press ctrl+Cto stop the process

        // 初始化Python
        Py_Initialize();
        // 检查初始化是否成功
        if (!Py_IsInitialized())
        {
            ICV_LOG_ERROR << "Python initializer Failed";
        }
        // 添加当前路径
        //把输入的字符串作为Python代码直接运行，返回0
        //表示成功，-1表示有错。大多时候错误都是因为字符串中有语法错误。
        PyRun_SimpleString("import sys");
        //PyRun_SimpleString("import numpy as np");

        // int result = PyRun_SimpleString("import numpy as np");
        // if (result != -1)
        // {
        //     printf("PYTHON USAGE OK!\n\n");
        // }

        PyRun_SimpleString("sys.path.append(\'.\')");
        data_demo_type1 input;

        ExecutePyFunction("pytest", "init",input, output);
        std::cout<<"PyInit Finished.\n";



        Register_Sub("data_174");
        Register_Sub("data_175");
        Register_Sub("data_176");
        Register_Sub("data_177");
        Register_Sub("data_178");
        Register_Sub("data_179");
        Register_Sub("data_17A");

        Register_Sub("msg_fix");
        Register_Sub("msg_vel");
        Register_Sub("obj_eq");

        Register_Pub("acc_str");

        timesync_ = new icv::icvSemaphore(1);
        icv_thread loop(bind(&PyFrontend::InnerLoop, this));
        threadsList_[0] = make_shared<std::thread>(&PyFrontend::func1, this);
        threadsList_[1] = make_shared<std::thread>(&PyFrontend::func2, this);
    }
    inline void InnerLoop()
    {
        while (true)
        {
            timesync_->Release();
            usleep(ICV_SLEEP_CYCLE);
        }
    }

    ~PyFrontend()
    {
        ICV_LOG_INFO << "C++ Frontend of Python script Stoped";
        //释放
        for (int i = 0; i < MAX_THREAD_COUNT; i++)
        {
            threadsList_[i]->join();
        }
        // 关闭Python
        Py_Finalize();
    }

 void ExecutePyFunction(std::string ScriptName, std::string FunctionName, data_demo_type1 &input, std::vector<double> &output)
    {
       output.clear();
        pName = PyUnicode_FromString(ScriptName.data());
        pModule = PyImport_Import(pName);
        if (!pModule)
        {

            ICV_LOG_ERROR << " Python script not found!";
        }

        PyObject *pDict = PyModule_GetDict(pModule);
        if (!pDict)
        {

            ICV_LOG_ERROR << "Python module missed";
        }

        PyObject  *pFunc, *pArgs, *pValue1, *pValue2, *pValue3, *pValue4, *pValue5, *pValue6, *pValue7, *pValue8, *pValue9, *pValue10, *pValue11, *pValue12;

	// Get the add method from the dictionary.
	pFunc = PyDict_GetItemString(pDict,FunctionName.data());
	pValue1 = PyLong_FromDouble(input.buf1);
	pValue2 = PyLong_FromDouble(input.buf2);
	pValue3 = PyLong_FromDouble(input.buf3);
	pValue4 = PyLong_FromDouble(input.buf4);
	pValue5 = PyLong_FromDouble(input.buf5);
	pValue6 = PyLong_FromDouble(input.buf6);
	pValue7 = PyLong_FromDouble(input.buf7);
	pValue8 = PyLong_FromDouble(input.buf8);
	pValue9 = PyLong_FromDouble(input.buf9);
	pValue10 = PyLong_FromDouble(input.buf10);
	pValue11 = PyLong_FromDouble(input.buf11);
	pValue12 = PyLong_FromDouble(input.buf12);
	//std::cout << "2" << endl;
	pArgs = PyTuple_Pack(12, pValue1,pValue2,pValue3,pValue4,pValue5,pValue6,pValue7,pValue8,pValue9,pValue10,pValue11,pValue12);
	

	PyTuple_SetItem(pArgs, 0, pValue1);
	PyTuple_SetItem(pArgs, 1, pValue2);
	PyTuple_SetItem(pArgs, 2, pValue3);
	PyTuple_SetItem(pArgs, 3, pValue4);
	PyTuple_SetItem(pArgs, 4, pValue5);
	PyTuple_SetItem(pArgs, 5, pValue6);
	PyTuple_SetItem(pArgs, 6, pValue7);
	PyTuple_SetItem(pArgs, 7, pValue8);
	PyTuple_SetItem(pArgs, 8, pValue9);
	PyTuple_SetItem(pArgs, 9, pValue10);
	PyTuple_SetItem(pArgs, 10, pValue11);
	PyTuple_SetItem(pArgs, 11, pValue12);

    PyObject* pResult = PyObject_CallObject(pFunc, pArgs);
	if (pResult == NULL)
		std::cout<<"Calling the add method failed.\n";
		
	for (int i =0; i<PyList_Size(pResult);i++)
	{
		PyObject* result = PyList_GetItem(pResult,i);
		output.emplace_back(PyFloat_AsDouble(result));
	}

        // //下面这段是查找函数test 并执行test
        // pFunc = PyDict_GetItemString(pDict, FunctionName.data());
        // if (!pFunc || !PyCallable_Check(pFunc))
        // {

        //     ICV_LOG_ERROR << "Function not found";
        // }
        // //申请python入参
        // pArgs = PyTuple_New(1);
        // //对python入参进行赋值; s代表char*格式, #代表传入指定长度
        // PyTuple_SetItem(pArgs, 0, Py_BuildValue("s#", input, size_of_input));
        // //执行函数
        // pResult = PyObject_CallObject(pFunc, pArgs);

        // //获取返回值
        // PyArg_Parse(pResult, "s", &output);

        //Py_DECREF(pName);
        Py_DECREF(pName);
        Py_DECREF(pArgs);
        Py_DECREF(pModule);
        Py_DECREF(pResult);

    }


    // void ExecutePyFunction(std::string ScriptName, std::string FunctionName, char *input, int size_of_input, char *&output)
    // {

    //     pName = PyBytes_FromString(ScriptName.data());
    //     pModule = PyImport_Import(pName);
    //     if (!pModule)
    //     {

    //         ICV_LOG_ERROR << " Python script not found!";
    //     }

    //     pDict = PyModule_GetDict(pModule);
    //     if (!pDict)
    //     {

    //         ICV_LOG_ERROR << "Python module missed";
    //     }

    //     //下面这段是查找函数test 并执行test
    //     pFunc = PyDict_GetItemString(pDict, FunctionName.data());
    //     if (!pFunc || !PyCallable_Check(pFunc))
    //     {

    //         ICV_LOG_ERROR << "Function not found";
    //     }
    //     //申请python入参
    //     pArgs = PyTuple_New(1);
    //     //对python入参进行赋值; s代表char*格式, #代表传入指定长度
    //     PyTuple_SetItem(pArgs, 0, Py_BuildValue("s#", input, size_of_input));
    //     //执行函数
    //     pResult = PyObject_CallObject(pFunc, pArgs);

    //     //获取返回值
    //     PyArg_Parse(pResult, "s", &output);

    //     Py_DECREF(pName);
    //     Py_DECREF(pArgs);
    //     Py_DECREF(pModule);
    // }
    void func1()
    {
        while (true)
        {
            // ICV_LOG_INFO << "子线程1演示：func1";
        }
    }
    void func2()
    {
        while (true)
        { //ICV_LOG_INFO << "子线程2演示：func2";
        }
    }
    virtual void Execute() override
    {

        icvSubscribe("data_174", &ANGL_DATA2);
        icvSubscribe("data_175", &WHEL_DATA2);
        icvSubscribe("data_176", &SPED_DATA2);
        icvSubscribe("data_177", &YAW_DATA2);
        icvSubscribe("data_178", &LAMP_DATA2);
        icvSubscribe("data_179", &GEAR_DATA2);
        icvSubscribe("data_17A", &WARN_DATA2);

        icvSubscribe("msg_fix", &FIX_DATA2);
        icvSubscribe("msg_vel", &VEL_DATA2);
        icvSubscribe("obj_eq", &OBJ_EQ2);

        

        angl_data2 = ANGL_DATA2.getvalue();
        whel_data2 = WHEL_DATA2.getvalue();
        sped_data2 = SPED_DATA2.getvalue();
        yaw_data2 = YAW_DATA2.getvalue();
        lamp_data2 = LAMP_DATA2.getvalue();
        gear_data2 = GEAR_DATA2.getvalue();
        warn_data2 = WARN_DATA2.getvalue();

        auto msg_vel2 = VEL_DATA2.getvalue();
        auto msg_fix2 = FIX_DATA2.getvalue();
        OBJ_40_EQ obj_eq2  = OBJ_EQ2.getvalue();

        ICV_LOG_DEBUG << "whel_data.FLWheelSpd :" << whel_data2.FLWheelSpd;
        ICV_LOG_DEBUG << "whel_data.FRWheelSpd :" << whel_data2.FRWheelSpd;
 
        ICV_LOG_DEBUG << "imu_data.FRWheelSpd :" <<msg_fix2.latitude;
        ICV_LOG_DEBUG << "imu_data.FRWheelSpd :" <<msg_fix2.longitude;
        


        // for (int i=0;i<40;i++){
        //     OBJ_EQ one_object=obj_eq2.obj_s[i];
        //     if(one_object.OBJ_ID!=0){
        //     std::cout 
        //                 << std::to_string(one_object.OBJ_ID) << " "
        //                 << std::to_string(one_object.OBJ_Object_Class) << " "
        //                 << std::to_string(one_object.OBJ_Class_Probability) << " "
        //                 << std::to_string(one_object.OBJ_Lat_Distance) << " "
        //                 << std::to_string(one_object.OBJ_Long_Distance) << " "
        //                 << std::to_string(one_object.OBJ_Relative_Lat_Velocity) << " "
        //                 << std::to_string(one_object.OBJ_Relative_Long_Velocity) << " "
        //                 << std::to_string(one_object.OBJ_Relative_Long_Acc) << " "
        //                 << std::to_string(one_object.OBJ_Abs_Lat_Acc) << " "
        //                 << std::to_string(one_object.OBJ_Abs_Long_Acc) << " "
        //                 << std::to_string(one_object.OBJ_Abs_Lat_Velocity) << " "
        //                 << std::to_string(one_object.OBJ_Abs_Long_Velocity) << " "
        //                 << std::to_string(one_object.OBJ_Abs_Acceleration) << " "
        //                 << std::to_string(one_object.OBJ_Height) << " "
        //                 << std::to_string(one_object.OBJ_Height_STD) << " "
        //                 << std::to_string(one_object.OBJ_Length) << " "
        //                 << std::to_string(one_object.OBJ_Length_STD) << " "
        //                 << std::to_string(one_object.OBJ_Width) << " "
        //                 << std::to_string(one_object.OBJ_Width_STD) << " "
        //                 << std::to_string(one_object.OBJ_Existence_Probability) << " "
        //                 << std::to_string(one_object.OBJ_Age_Seconds) << " "
        //                 << std::to_string(one_object.OBJ_Motion_Orientation) << " "
        //                 << std::to_string(one_object.OBJ_Motion_Status) << " "
        //                 << std::to_string(one_object.OBJ_Measuring_Status) << " "
        //                 << std::to_string(one_object.OBJ_Motion_Category) << " "
        //                 << std::to_string(one_object.OBJ_Object_Age) << "\n";}
        // }

             //创建结构体
        data_demo_type1 input;
        data_demo_type2 output_vehicle;

        Eigen::Vector3d origin_lla =  Eigen::Vector3d(144,144,1);
                   ICV_LOG_INFO << "  111:" ;

        Eigen::Vector3d position_lla =  Eigen::Vector3d(144,144,1);//Eigen::Vector3d(msg_fix2.latitude,msg_fix2.longitude,msg_fix2.altitude);   //纬度，经度，高度
                          ICV_LOG_INFO << "  222:" ;

        Eigen::Vector3d position_enu;
                   ICV_LOG_INFO << "  333:" ;

        ConvertLLAToENU(origin_lla, position_lla, &position_enu);

           float32 positionx = position_enu[0];
           float32 positiony = position_enu[1];
           float32 positionz = position_enu[2];
           ICV_LOG_INFO << "  longitude  positionx:" << positionx;

        // ego vehicle
        input.buf1 = 0;// positionx;
        input.buf2 = 0;//positiony;
        input.buf3 = 10;//sped_data2.VehSpd / 0.05625;   //km/h
        input.buf4 = 0;//msg_fix2.heading*1e-6;    //  1e-6 rad
        input.buf5 = 10;//msg_vel2.twist.twist.linear.x;
        input.buf6 = 0;//msg_vel2.twist.twist.linear.y;


        

  
        // ego vehicle
        // input.buf1 = 1;
        // input.buf2 = 2;
        // input.buf3 = 2;
        // input.buf4 = 3;
        // input.buf5 = 3;
        // input.buf6 = 3;

        // // env vehicle1
        // input.buf7 = 3;
        // input.buf8 = 3;
        // input.buf9 = 3;
        // input.buf10 = 3;
        // input.buf11 = 3;
        // input.buf12 = 3;

        // // env vehicle2
        // input.buf13 = 3;
        // input.buf14 = 3;
        // input.buf15 = 3;
        // input.buf16 = 3;
        // input.buf17 = 3;
        // input.buf18 = 3;
        // strcpy(input.buf3, "kjac");

        //打包成byte*
        // char *byInput = new char(sizeof(input));
        // memcpy(byInput, &input, sizeof(input));

        // char *pRsp;
        ExecutePyFunction("pytest", "pnc", input, output);
        //转成结构体
        //data_demo_type1 *pstRsp = (data_demo_type1 *)pRsp;
        output_vehicle.acc = output[1];
        output_vehicle.steer = output[2];
printf("\n-----------主线程：c++层接收py返回:buf1:%f, buf2:%f\n",output[1], output[2]);
        output_veh.setvalue(output_vehicle);
         icvPublish("acc_str", &output_veh);

        

    }

private:
    PyObject *pName, *pModule, *pDict, *pArgs, *pResult, *pFunc;
    std::array<std::shared_ptr<std::thread>, MAX_THREAD_COUNT> threadsList_;
    icv::icvSemaphore *timesync_;
    std::vector<double> output;
    data::icvStructureData<OBJ_40_EQ>  OBJ_EQ2;
     //icvNavSatFix msg_fix_t2;
    icvNavSatFix FIX_DATA2;
    icvTwistWithCovarianceStamped VEL_DATA2;
    icvImu IMU_DATA2;
    icvOdometry msg_odom_t2;    
    
    icvdata_demo_type2 output_veh;

	SCU_IPC_0x174  ANGL_DATA2;
	CanFrame_SCU_IPC_1_0x174 angl_data2;
    SCU_IPC_0x175  WHEL_DATA2;
	CanFrame_SCU_IPC_2_0x175 whel_data2;
    SCU_IPC_0x176  SPED_DATA2;
	CanFrame_SCU_IPC_3_0x176 sped_data2;
    SCU_IPC_0x177  YAW_DATA2;
	CanFrame_SCU_IPC_4_0x177 yaw_data2;
    SCU_IPC_0x178 LAMP_DATA2;
	CanFrame_SCU_IPC_5_0x178 lamp_data2;
    SCU_IPC_0x179  GEAR_DATA2;
	CanFrame_SCU_IPC_6_0x179 gear_data2;
    SCU_IPC_0x17A  WARN_DATA2;
	CanFrame_SCU_IPC_7_0x17A warn_data2;
    
};

ICV_REGISTER_FUNCTION(PyFrontend)

#endif //
