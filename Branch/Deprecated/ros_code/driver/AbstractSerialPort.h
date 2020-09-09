#ifndef ABSTRACTSERIALPORT_H
#define ABSTRACTSERIALPORT_H

class QString;

namespace pacpus {

class AbstractSerialPort
{
public:
    AbstractSerialPort(QString name);
    virtual ~AbstractSerialPort();
};

} // namespace pacpus

#endif // ABSTRACTSERIALPORT_H
