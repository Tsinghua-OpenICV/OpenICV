#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"

using namespace icv;
using namespace icv::core;
using namespace icv::data;

class TimesFunction : public icvFunction
{
public:
    TimesFunction(icv_shared_ptr<const icvMetaData> info) : icvFunction(2, 1, info) {}
    TimesFunction() : TimesFunction(ICV_NULLPTR) {}



    virtual void Execute(icvDataObject** inData, icvDataObject** outData) ICV_OVERRIDE
    {
        double data1 = *static_cast<icvDoubleData*>(inData[0]);
        double data2 = *static_cast<icvDoubleData*>(inData[1]);
        outData[0]->As<icvDoubleData>() = data1 * data2;

        ICV_LOG_TRACE << "Times data from " << data1 << ", " << data2 << " to " << data1 * data2;
    }
private:
    std::vector<double> _coeffs;
};

ICV_REGISTER_FUNCTION(TimesFunction)
