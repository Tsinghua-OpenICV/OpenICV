#include "OpenICV/Basis/icvRandomSource.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeOutput.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"

#include <random>

using namespace icv::data;

namespace icv { namespace function
{
    ICV_REGISTER_FUNCTION(icvRandomSource)

    ICV_CONSTEXPR char KEY_INTERVAL[] = "interval";

    icvRandomSource::icvRandomSource() : icvRandomSource(ICV_NULLPTR) {}
    icvRandomSource::icvRandomSource(icv_shared_ptr<const icvMetaData> info)
        : icvFunction(0, 1, info)
    {
        if (_information.Contains(KEY_INTERVAL))
        {
            _interval = _information.GetInteger(KEY_INTERVAL);
            _information.Remove(KEY_INTERVAL);
        }
        tempdata= new icv::data::icvInt64Data();tempdata->Reserve();
    }
    //  void icvRandomSource::ConfigurateInput(std::vector<icvNodeInput*>& inputPorts) 
    // {
	// 		ini_Input(inputPorts,0);

    // }
    // void icvRandomSource::ConfigurateOutput(std::vector<icvNodeOutput*>& outputPorts)
    // {
    //    // CheckDataType<icvDoubleData>(outputPorts[0],sizeof(double));

    //    ini_Output(outputPorts,1);

	// 		icvInt64Data *randnumber=new icvInt64Data();
	// 		outputPorts[0]->SetDataObject(randnumber);
	// 		ICV_LOG_TRACE << "Instantiated DataObjec at output";
    // }

    void icvRandomSource::Execute(std::vector<icvNodeInput*>& inputPorts, std::vector<icvNodeOutput*>& outputPorts)
    {


        _sleepFunc(icv_chrono::duration_cast<Duration64>(icv_chrono::milliseconds(_interval)));

        int data = rand() % 256;
     //   outData[0]->As<icvDoubleData>() = data;

        tempdata->setoutvalue(data);

		Send_Out(tempdata,0);
        //ICV_LOG_INFO << "generate data: " << data;
      //   ICV_LOG_INFO <<"flag :"<<outputPorts[0]-> get_flag();

        //setout
    }
}}