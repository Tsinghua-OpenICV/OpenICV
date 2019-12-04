/*********************************************************************
// created:    2005/09/02 - 8:01
// filename:   win32SerialPort.h
//
// author:     Gerald Dherbomez
//
// version:    $Id: win32SerialPort.h 1277 2013-01-10 16:49:06Z bonnetst $
//
// purpose:    This class defines a Windows (32bits) driver for serial port
*********************************************************************/

#ifndef WIN32SERIALPORT_H
#define WIN32SERIALPORT_H

#ifdef WIN32
#   include <windows.h>
#endif

#include "Pacpus/kernel/pacpus.h"
#include "dataFrame.h"

#include <QByteArray>
#include <QMutex>
#include <QQueue>
#include <QThread>
#include <string>

typedef struct {
    unsigned int length;      // Deprecated, needed by some legacy code
    road_time_t t;
    road_timerange_t tr;
    QByteArray data;
} FRAME;

class Win32SerialPort
    : public QThread
{
    Q_OBJECT

public:
    enum PpsSense { RISING, FALLING, BOTH };


    Win32SerialPort(QString name);
    ~Win32SerialPort();

    /*!< returns the number of items in the queue */
    int numberOfFrames();

    /*!< returns the first frame in the queue without removing it */
    FRAME* firstFrame();

    /*!< remove the first data frame from the queue */
    int removeFirstFrame();

    /*!< return the next dataFrame in the list and removes it */
    FRAME* getNextFrame();

    /*!< open the port 'name'
    return true if success
    */
    bool openPort(const char * name);

    /*!< close the port
    return true if success
    */
    int closePort();

    /*!< configure the port
    return true if success
    */
    int configurePort(unsigned long baudRate, unsigned char byteSize, unsigned char parity, unsigned char stopBits);

    /*!< set the mode of usage of the port
    */
    void setMode(const DWORD overlappedModeAttribute = 0); // FILE_FLAG_OVERLAPPED

    void setPpsSense(enum PpsSense);

    BOOL THREAD_ALIVE;

Q_SIGNALS:
    void newDataAvailable(int);

protected:
    QString componentName;

private:
    void setPortName(const char * name);
    bool setPort(long baudrate,char parity,int byteSize,int stopBits );

    void processIncomingEvent();
    void processIncomingData();
    int readBuffer(char * buffer, int maxLength);
    DWORD nbBytesToRead();

    void dumpParameters(QString filename);

    void run();

private:
    HANDLE handlePort;

    DWORD overlappedMode;
    OVERLAPPED overlappedStructure;
    COMMCONFIG commConfigStructure;
    COMMTIMEOUTS commTimeoutsStructure;
    UINT inputBufferSize;
    UINT outputBufferSize;

    DWORD baudRate_;
    BYTE byteSize_;
    BYTE parity_;
    BYTE stopBits_;

    PpsSense ppsSense_;

    DWORD evtMask;
    COMSTAT comStat;
    DWORD comErr;
    DWORD comOperationStatus;

    QByteArray receivedBuffer_;
    int numberBytesToRead;
    int numberBytesRead;
    BOOL ringIndicatorDetected;

    QQueue<FRAME *> dataFrameQueue;

    std::string portName;

    // local variables
    road_time_t t_;

    QMutex frameLock_;
};

#endif // WIN32SERIALPORT_H
