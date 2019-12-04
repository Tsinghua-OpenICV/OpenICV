#include <boost/python.hpp>

#define ICV_CONFIG_POINTERS
#include "OpenICV/Core/icvConfig.h"
#undef ICV_CONFIG_POINTERS

#include "OpenICV/Core/icvObject.h"
#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvNode.h"

using namespace icv::core;
using namespace boost::python;

class icvFunctionWrap : icvFunction, public wrapper<icvFunction>
{
    void Execute(icvDataObject** inData, icvDataObject** outData) ICV_OVERRIDE
    { this->get_override("Execute")(); }


};

BOOST_PYTHON_MODULE(_icvCore)
{
    class_<icvObject>("icvObject");

    class_<icvFunctionWrap, bases<icvObject>>("icvFunction")
        .def("Execute", pure_virtual(&icvFunction::Execute))
        .def("RegisterSuspend", &icvFunction::RegisterSuspend);
    register_ptr_to_python<icv_shared_ptr<icvFunction>>();
}
