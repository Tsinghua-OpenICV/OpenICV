#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"

#include <boost/thread/thread.hpp>

using namespace icv;
using namespace icv::core;

class PolynomialFunction : public icvFunction
{
public:
    PolynomialFunction(icv_shared_ptr<const icvMetaData> info) : icvFunction(1, 1, info)
    {
        if (_information.Contains("coefficients"))
        {
            auto arr = _information.GetArray("coefficients");
            for (int i = 0; i < arr.Size(); i++)
                _coeffs.push_back(arr.GetDecimal(i));
            _information.Remove("coefficients");
        }
    }
    PolynomialFunction() : PolynomialFunction(ICV_NULLPTR) {}



    virtual void Execute(icvDataObject** inData, icvDataObject** outData) ICV_OVERRIDE
    {
        double data = *static_cast<icvDoubleData*>(inData[0]);
        double result = data;
        for (double coeff : _coeffs) result = result * coeff + data;
        outData[0]->As<icvDoubleData>() = result;

        ICV_LOG_TRACE << "Filter Data from " << data << " to " << result;
    }
private:
    std::vector<double> _coeffs;
};

ICV_REGISTER_FUNCTION(PolynomialFunction)

