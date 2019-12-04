#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"

using namespace icv;
using namespace icv::core;
using namespace icv::data;

class AddFunction : public icvFunction
{
public:
    AddFunction(icv_shared_ptr<const icvMetaData> info) : icvFunction(2, 1) {}
    AddFunction() : AddFunction(ICV_NULLPTR) {}



    virtual void Execute(icvDataObject** inData, icvDataObject** outData) ICV_OVERRIDE
    {
        double data1 = inData[0]->As<icvDoubleData>();
        double data2 = inData[1]->As<icvDoubleData>();
        outData[0]->As<icvDoubleData>() = data1 + data2;

        ICV_LOG_TRACE << "Add data from " << data1 << ", " << data2 << " to " << data1 + data2;
    }
private:
    std::vector<double> _coeffs;
};

ICV_REGISTER_FUNCTION(AddFunction)

