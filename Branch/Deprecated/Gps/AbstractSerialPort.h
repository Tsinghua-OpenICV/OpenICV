#ifndef ABSTRACTSERIALPORT_H
#define ABSTRACTSERIALPORT_H
#include "OpenICV/Core/icvFunction.h"


namespace icv {

class AbstractSerialPort
{
public:
    AbstractSerialPort( ){};
    virtual ~AbstractSerialPort(){};
};

} // namespace pacpus

#endif // ABSTRACTSERIALPORT_H
