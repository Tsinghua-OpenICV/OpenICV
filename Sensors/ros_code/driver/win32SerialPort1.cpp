/*********************************************************************
// created:     2005/05/30 - 15:07
// filename:    win32SerialPort.cpp
//
// author:      Gerald Dherbomez
//
// version:     $Id: win32SerialPort.cpp 1278 2013-01-16 16:40:11Z bonnetst $
//
// purpose:     Implementation of the win32SerialPort class.
//
// todo:        - Manage infinite waiting when the port is not correctly
//              opened or where no data available: reinit the port
//              - add a function to write data
//              - provide 3 possibilities to use the serial port:
//                with event overlapped           ok
//                with event non overlapped       ok
//                without event directly          todo
//              - look at why the port is not opened correctly at the
//              beginning
*********************************************************************/

#include "Win32SerialPort.h"

#include "Pacpus/kernel/Log.h"

#include <cassert>
#include <iostream>
#include <QFile>
#include <QTextStream>
#include <sstream>

DECLARE_STATIC_LOGGER("pacpus.base.Win32SerialPort");

using namespace std;

static const int PORT_NAME_MAXIMUM_LENGTH = 20;

static const int XON  = 17;
static const int XOFF = 19;

//////////////////////////////////////////////////////////////////////////
// Constructor
//////////////////////////////////////////////////////////////////////////
Win32SerialPort::Win32SerialPort(QString name)
{
    componentName = name;

    // default values
    baudRate_ = CBR_38400;
    byteSize_ = 8;
    parity_ = NOPARITY;
    stopBits_ = ONESTOPBIT;

    THREAD_ALIVE = TRUE;
    numberBytesToRead = 0;
    ringIndicatorDetected = FALSE;
    comOperationStatus = 0;

    ppsSense_ = RISING;

    t_=0;

    LOG_INFO("The win32 serial port " << componentName << " was created");
}

//////////////////////////////////////////////////////////////////////////
// Destructor
//////////////////////////////////////////////////////////////////////////
Win32SerialPort::~Win32SerialPort()
{
    LOG_INFO("The win32 serial port " << componentName << " was destroyed");
}

//////////////////////////////////////////////////////////////////////////
// Set the mode of the serial port driver:
// - overlapped: it is the asynchronous mode (non blocking).
// Use FILE_FLAG_OVERLAPPED in parameter
// - non overlapped: it is the synchronous mode (blocking).
// Pass 0 in parameter
//
// Warning: the mode has to be chosen before opening the COM port
//////////////////////////////////////////////////////////////////////////
void Win32SerialPort::setMode(const DWORD overlappedModeAttribute)
{
    // reset overlapped structure
    memset(&overlappedStructure,0,sizeof(overlappedStructure));

    overlappedMode = overlappedModeAttribute;
}

//////////////////////////////////////////////////////////////////////////
// Open the port 'name'. 'name' follows the pattern COMx with x the port
// number that you want to open
//////////////////////////////////////////////////////////////////////////
bool Win32SerialPort::openPort(const char * name)
{
    setPortName(name);

    handlePort = CreateFile(
        /* lpFileName = */ portName.c_str(),
        /* dwDesiredAccess = */ GENERIC_READ | GENERIC_WRITE,
        /* dwShareMode = */ 0,
        /* lpSecurityAttributes = */ NULL,
        /* dwCreationDisposition = */ OPEN_EXISTING,
        /* dwFlagsAndAttributes = */ overlappedMode,
        /* hTemplateFile = */ NULL
        );

    if (handlePort == INVALID_HANDLE_VALUE) {
        LOG_ERROR("CreateFile failed with error "<< GetLastError());
        return false;
    }
    return true;
}

//////////////////////////////////////////////////////////////////////////
// Close the active COM port
//////////////////////////////////////////////////////////////////////////
int Win32SerialPort::closePort()
{
    if (!CloseHandle(handlePort)) {
        LOG_ERROR("CloseHandle failed with error " << GetLastError());
        return 0;
    }
    return 1;
}

//////////////////////////////////////////////////////////////////////////
// Set the name of the COM port that will be used
// Called by openPort() function
//////////////////////////////////////////////////////////////////////////
void Win32SerialPort::setPortName(const char * name)
{
    // the string "\\.\" (unescaped) is necessary to open a port COM above 9
    portName = "\\\\.\\" + string(name);
    LOG_DEBUG("portName = '" << portName << "'");
    assert(PORT_NAME_MAXIMUM_LENGTH >= portName.length());
}

//////////////////////////////////////////////////////////////////////////
// Configure the main port parameters: baudrate, parity, byte size and
// number of stop bits
// called by configurePort - private member
//////////////////////////////////////////////////////////////////////////
bool Win32SerialPort::setPort(long baudrate, char parity, int byteSize, int stopBits)
{
    if (!handlePort) {
        return false;
    }

    baudRate_ = baudrate;
    parity_ = parity;
    byteSize_ = byteSize;
    stopBits_ = stopBits;

    stringstream baudSs;
    baudSs << (const char *) "baud=" << baudrate;
    baudSs << (const char *) "parity=" << parity;
    baudSs << (const char *) "data=" << byteSize;
    baudSs << (const char *) "stop=" << stopBits;
    string szBaud = baudSs.str();

    //    LOG_DEBUG("COMM definition string = " << szBaud);

    int result;
    if (result=GetCommState(handlePort,&commConfigStructure.dcb)) {
        commConfigStructure.dcb.fRtsControl = RTS_CONTROL_DISABLE;
        if (BuildCommDCB(szBaud.c_str(), &commConfigStructure.dcb)) {
            result=SetCommState(handlePort, &commConfigStructure.dcb);
        }
    }
    if ( result < 0 ) {
        LOG_WARN("Failed to set com state. Error: " << GetLastError());
    }

    PurgeComm(handlePort, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
    return result > 0;
}

void Win32SerialPort::dumpParameters(QString filename)
{
    // get the current DCB (device-control block) structure
    GetCommState(handlePort, &commConfigStructure.dcb);

    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        LOG_ERROR("Cannot open file: " << filename);
        return;
    }

    QTextStream out(&file);

    // write to file
    out << "commConfigStructure.dcb.DCBlength " << commConfigStructure.dcb.DCBlength << "\n";
    out << "commConfigStructure.dcb.BaudRate " << commConfigStructure.dcb.BaudRate << "\n";
    out << "commConfigStructure.dcb.ByteSize " << commConfigStructure.dcb.ByteSize << "\n";
    out << "commConfigStructure.dcb.Parity " << commConfigStructure.dcb.Parity << "\n";
    out << "commConfigStructure.dcb.StopBits " << commConfigStructure.dcb.StopBits << "\n";
    out << "commConfigStructure.dcb.fBinary " << commConfigStructure.dcb.fBinary << "\n";
    out << "commConfigStructure.dcb.fParity " << commConfigStructure.dcb.fParity << "\n";
    out << "commConfigStructure.dcb.fOutX " << commConfigStructure.dcb.fOutX << "\n";
    out << "commConfigStructure.dcb.fInX " << commConfigStructure.dcb.fInX << "\n";
    out << "commConfigStructure.dcb.fOutxCtsFlow " << commConfigStructure.dcb.fOutxCtsFlow << "\n";
    out << "commConfigStructure.dcb.fOutxDsrFlow " << commConfigStructure.dcb.fOutxDsrFlow << "\n";
    out << "commConfigStructure.dcb.fRtsControl " << commConfigStructure.dcb.fRtsControl << "\n";
    out << "commConfigStructure.dcb.fNull " << commConfigStructure.dcb.fNull << "\n";
    out << "commConfigStructure.dcb.XonChar " << commConfigStructure.dcb.XonChar << "\n";
    out << "commConfigStructure.dcb.XoffChar " << commConfigStructure.dcb.XoffChar << "\n";
    out << "commConfigStructure.dcb.XonLim " << commConfigStructure.dcb.XonLim << "\n";
    out << "commConfigStructure.dcb.XoffLim " << commConfigStructure.dcb.XoffLim << "\n";
    out << "commConfigStructure.dcb.fAbortOnError " << commConfigStructure.dcb.fAbortOnError << "\n";
    out << "commConfigStructure.dcb.fDtrControl " << commConfigStructure.dcb.fDtrControl << "\n";
    out << "commConfigStructure.dcb.fDsrSensitivity " << commConfigStructure.dcb.fDsrSensitivity << "\n";

    GetCommTimeouts( handlePort, &commTimeoutsStructure);

    out << "commTimeoutsStructure.ReadIntervalTimeout " << commTimeoutsStructure.ReadIntervalTimeout << "\n";
    out << "commTimeoutsStructure.ReadTotalTimeoutMultiplier " << commTimeoutsStructure.ReadTotalTimeoutMultiplier << "\n";
    out << "commTimeoutsStructure.ReadTotalTimeoutConstant " << commTimeoutsStructure.ReadTotalTimeoutConstant << "\n";
    out << "commTimeoutsStructure.WriteTotalTimeoutMultiplier " << commTimeoutsStructure.WriteTotalTimeoutMultiplier << "\n";
    out << "commTimeoutsStructure.WriteTotalTimeoutConstant " << commTimeoutsStructure.WriteTotalTimeoutConstant << "\n";

    file.close();
}

//////////////////////////////////////////////////////////////////////////
// Configure the port with Windows requested parameters
// Call setPort() function with provided parameters
//////////////////////////////////////////////////////////////////////////
int Win32SerialPort::configurePort(unsigned long baudRate, unsigned char byteSize, unsigned char parity, unsigned char stopBits)
{
    bool fSuccess;
    DCB dcb;
    dumpParameters("serialportparameters_before.txt");

    printf("%d %d %d %d \n", baudRate, byteSize, parity, stopBits);

    inputBufferSize = 32;
    outputBufferSize = 32;

    SetupComm( handlePort, inputBufferSize, outputBufferSize );
    //::GetCommMask( handlePort, &evtMask );
    //::SetCommMask( handlePort, 0 );

    //  Initialize the DCB structure.
    SecureZeroMemory(&dcb, sizeof(DCB));
    dcb.DCBlength = sizeof(DCB);


    // get the current DCB (device-control block) structure
    fSuccess = GetCommState(handlePort, &dcb);
    if (!fSuccess) {
        //  Handle the error.
		DWORD errorme=GetLastError();
        LOG_ERROR("GetCommState failed with error " <<errorme );
        return 0;
    }
	else LOG_WARN("GetCommState succeed");
    // fill the DCB structure
    //commConfigStructure.dcb.DCBlength =	sizeof(DCB);    // G
    dcb.BaudRate = baudRate;					// port speed (CBR_xxxx)
    dcb.ByteSize = byteSize;					// number of bits/byte, 4-8
    dcb.Parity = parity ;					    // 0-4=no,odd,even,mark,space
    dcb.StopBits = stopBits;					// 0,1,2 = 1, 1.5, 2


    /* Rest of the dcb structure */
    dcb.fBinary = TRUE;
    dcb.fParity = TRUE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fTXContinueOnXoff = TRUE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fErrorChar = FALSE;
    dcb.fNull = FALSE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fAbortOnError = FALSE;

    /*
    dcb.BaudRate = CBR_4800;					// port speed (CBR_xxxx)
    dcb.ByteSize = 8;					// number of bits/byte, 4-8
    dcb.Parity = NOPARITY ;					    // 0-4=no,odd,even,mark,space
    dcb.StopBits = ONESTOPBIT;					// 0,1,2 = 1, 1.5, 2
    */
    /*commConfigStructure.dcb.fBinary = 1;						      // binary mode on (off not supported by win32 API)
    commConfigStructure.dcb.fParity = 0;					        // Consider parity or not ?
    commConfigStructure.dcb.fOutX = FALSE;						  // XON/XOFF out flow control
    commConfigStructure.dcb.fInX = FALSE;						    // XON/XOFF in flow control
    commConfigStructure.dcb.fOutxCtsFlow = 0;					  	// CTS output flow control
    commConfigStructure.dcb.fOutxDsrFlow = 0;						  // DSR output flow control
    commConfigStructure.dcb.fRtsControl = RTS_CONTROL_DISABLE;		// RTS flow control type

    commConfigStructure.dcb.fNull = 0;						        // Specifies whether null bytes are discarded   //G
    commConfigStructure.dcb.XonChar = XON;
    commConfigStructure.dcb.XoffChar = XOFF;
    commConfigStructure.dcb.XonLim = (WORD) ((inputBufferSize) / 4);
    commConfigStructure.dcb.XoffLim = (WORD) ((outputBufferSize) / 4);
    commConfigStructure.dcb.fAbortOnError = 1;	  		// need rest on error (use ClearCommError)  //G
    commConfigStructure.dcb.fDtrControl = DTR_CONTROL_DISABLE;		// DTR flow control type   //G
    commConfigStructure.dcb.fDsrSensitivity =	FALSE;		// Don't consider DSR line   // G
    */
    // apply the DCB structure for RS-232 serial device
    fSuccess = SetCommState(handlePort,&dcb);
    if (!fSuccess)
    {
        //  Handle the error.
        printf ("SetCommState failed with error %d.\n", GetLastError());
        return 0;
    }

    fSuccess = GetCommTimeouts( handlePort, &commTimeoutsStructure);
    if (!fSuccess)
    {
        //  Handle the error.
        printf ("GetCommTimeouts failed with error %d.\n", GetLastError());
        return 0;
    }
	

    if (overlappedMode) {
        commTimeoutsStructure.ReadIntervalTimeout = MAXDWORD;
        commTimeoutsStructure.ReadTotalTimeoutMultiplier = 0;
        commTimeoutsStructure.ReadTotalTimeoutConstant = 0;
        commTimeoutsStructure.WriteTotalTimeoutMultiplier = 0;
        commTimeoutsStructure.WriteTotalTimeoutConstant = 0;
    } else {
        commTimeoutsStructure.ReadIntervalTimeout = 2;
        commTimeoutsStructure.ReadTotalTimeoutMultiplier = 10;
        commTimeoutsStructure.ReadTotalTimeoutConstant = 20;
        commTimeoutsStructure.WriteTotalTimeoutMultiplier = 10;
        commTimeoutsStructure.WriteTotalTimeoutConstant = 20;
    }

    SetCommTimeouts( handlePort, &commTimeoutsStructure );

    // specify the set of events to be monitored
    //if (!SetCommMask(handlePort, EV_BREAK | EV_CTS | EV_DSR | EV_ERR | EV_RING | EV_RLSD | EV_RXFLAG | EV_RXCHAR | EV_TXEMPTY) )
    if (!SetCommMask(handlePort, EV_ERR | EV_RLSD | EV_RXFLAG | EV_RXCHAR) )
    {
        //  Handle the error.
        printf ("SetCommMask failed with error %d.\n", GetLastError());
        return 0;
    }
    //setPort(baudRate, parity, byteSize, stopBits);

    if (overlappedMode == FILE_FLAG_OVERLAPPED)
    {
        // Create an event object for use by WaitCommEvent.
        overlappedStructure.hEvent = CreateEvent(
            NULL,   // default security attributes
            TRUE,   // (true = manual - false = auto) reset event
            FALSE,  // initial state : not signaled
            NULL    // no name
            );

        if (overlappedStructure.hEvent == NULL)
            qWarning("overlapped event is not created\n");

        // Intialize the rest of the OVERLAPPED structure to zero.
        overlappedStructure.Internal = 0;
        overlappedStructure.InternalHigh = 0;
        overlappedStructure.Offset = 0;
        overlappedStructure.OffsetHigh = 0;
    }

    dumpParameters("serialportparameters_after.txt");

    return 1;
}

//////////////////////////////////////////////////////////////////////////
// Main loop in charge of processing incoming events from the driver
// There are 2 main functions:
//  - processIncomingEvent that detects the nature of the event : RXCHAR
//  , RING ... and get data if available
//  - processIncomingData that processes the data got by the first function
//  and transmits the stream to the decoding module
//////////////////////////////////////////////////////////////////////////
void Win32SerialPort::run()
{
    // GD 31/08/2007 - deja appelé dans GpsComponent::run()
    //configurePort(baudRate_, byteSize_, parity_, stopBits_);

    PurgeComm(handlePort, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

    while (THREAD_ALIVE)
    {
        switch(overlappedMode)
        {
        case 0:                 // non overlapped operation
            if (WaitCommEvent(handlePort, &evtMask, 0))
            {
                t_ = road_time();                     // datation
                processIncomingEvent();               // processing
                processIncomingData();
            }
            else
            {
                qWarning("WaitCommEvent failed with error %d.\n", GetLastError());
                msleep(50);
            }
            break;

        case FILE_FLAG_OVERLAPPED:        // overlapped operation
            if (WaitCommEvent(handlePort, &evtMask, &overlappedStructure)) {
                t_ = road_time();                     // datation
                processIncomingEvent();
                processIncomingData();
            } else {
                // datation
                comOperationStatus = GetLastError();
                if (ERROR_IO_PENDING == comOperationStatus) {
                    if (WaitForSingleObject(overlappedStructure.hEvent, INFINITE) == WAIT_OBJECT_0) {
                        t_ = road_time();
                        processIncomingEvent();
                        processIncomingData();
                    } 
                }
				else LOG_WARN("WaitForSingleObject failed with error " << GetLastError());
            }

            break;
        default:
            LOG_WARN("Problem for acquiring serial port");

        } // END SWITCH(OVERLAPPEDMODE)
    } // END WHILE

    LOG_INFO("Port '" << portName << "' (Win32SerialPort) thread is stopped");
}

//////////////////////////////////////////////////////////////////////////
// It is the function responsible to process new events
// In normal conditions EV_RXCHAR is received to alert that new data can
// be read
// PPS : RING (pin 9) or DCD (pin 1, aka RLSD) or DSR (pin 6)
//////////////////////////////////////////////////////////////////////////
void Win32SerialPort::processIncomingEvent() {
    numberBytesRead = 0;
    unsigned long modemStatus;

    GetCommModemStatus(handlePort, &modemStatus);
    if ( evtMask & EV_BREAK ) {
        //printf("%s EV_BREAK\n",portName);
    }

    if ( evtMask & EV_CTS ) {
        //printf("%s EV_CTS\n",portName);
    }

    if ( evtMask & EV_DSR ) {
        //printf("%s EV_DSR\n",portName);
        if ((!(modemStatus & MS_DSR_ON)) & (ppsSense_ == RISING)) {
            ringIndicatorDetected = TRUE;
        }
    }

    if ( evtMask & EV_ERR ) {
        //printf("%s EV_ERR\n",portName);
    }

    if ( evtMask & EV_RING ) {
        //printf("%s EV_RING\n", portName);
        if ((!(modemStatus & MS_RING_ON)) & (ppsSense_ == RISING)) {
            ringIndicatorDetected = TRUE;
        }
    }

    if ( evtMask & EV_RLSD ) {
        //printf("%s EV_RLSD\n",portName);
        if ((!(modemStatus & MS_RLSD_ON)) & (ppsSense_ == RISING)) {
            ringIndicatorDetected = TRUE;
        }
    }

    if ( evtMask & EV_RXCHAR ) {
		//printf("%s EV_RXCHAR\n",portName);
        if ((numberBytesToRead = nbBytesToRead()) > 0) {
            receivedBuffer_.resize(numberBytesToRead);
            numberBytesRead = readBuffer(receivedBuffer_.data(),numberBytesToRead);
        }
        receivedBuffer_.truncate(numberBytesRead);
    }


    if ( evtMask & EV_RXFLAG ) {
        printf("%s EV_RXFLAG\n",portName);
	}

        if ( evtMask & EV_TXEMPTY )
            //printf("%s EV_TXEMPTY\n",portName);

            evtMask = 0;
    // manual reset of overlapped event
    if (overlappedMode)
        ResetEvent(overlappedStructure.hEvent);
}



//////////////////////////////////////////////////////////////////////////
// Process the data received by the processIncomingEvent() function
// It may be either bytes (data) or a ring indicator signal (PPS for GPS signal)
//////////////////////////////////////////////////////////////////////////
void Win32SerialPort::processIncomingData() {

    if (numberBytesRead > 0)              // data frame
    {
        FRAME * frame = new FRAME;
        frame->t = t_;
        frame->tr = 0;
        frame->data = receivedBuffer_;

        frameLock_.lock();
        frame->length = frame->data.size();
        dataFrameQueue.enqueue(frame);
        frameLock_.unlock();
        emit newDataAvailable(1);
    }

    if (ringIndicatorDetected == TRUE)
    {
        ringIndicatorDetected = FALSE;
        FRAME * frame = new FRAME;
        frame->t = t_;
        frame->tr = 0;
        frame->data = "PPS";
        frameLock_.lock();
        frame->length = frame->data.size();
        dataFrameQueue.enqueue(frame);
        frameLock_.unlock();
        emit newDataAvailable(1);
    }
	cout<<"WIN32 proceeding"<<endl;
    // re-initialization
    t_ = 0;
    numberBytesToRead = 0;
}


//////////////////////////////////////////////////////////////////////////
// Read 'maxLength' bytes on the port and copy them in buffer
// return the number of bytes read
//////////////////////////////////////////////////////////////////////////
int Win32SerialPort::readBuffer(char *buffer, int maxLength)
{
    int countread;
    if (overlappedMode) {
        if (!ReadFile( handlePort,buffer,maxLength,(DWORD*)&countread,&overlappedStructure))
            qWarning("ReadFile failed with error %d.\n", GetLastError() );
    } else {
        if (!ReadFile( handlePort,buffer,maxLength,(DWORD*)&countread,0))
            qWarning("ReadFile failed with error %d.\n", GetLastError() );
    }

    return countread;
}

//////////////////////////////////////////////////////////////////////////
// return the number bytes that are available on the port
//////////////////////////////////////////////////////////////////////////
DWORD Win32SerialPort::nbBytesToRead()
{
    if (!ClearCommError( handlePort, &comErr, &comStat ))
        qWarning("ClearCommError failed with error %d\n",GetLastError());
    return comStat.cbInQue;
}

//////////////////////////////////////////////////////////////////////////
// return a pointer to the first frame in the queue and removes it
// The reader is responsible for deleting the frame.
//////////////////////////////////////////////////////////////////////////
FRAME * Win32SerialPort::getNextFrame()
{
    FRAME *frame = NULL;
    frameLock_.lock();
    if (!dataFrameQueue.isEmpty()) {
        frame = dataFrameQueue.dequeue();
    }
    frameLock_.unlock();
    return frame;
}

////////////////////////////////////////////////////////////////////////
// Old interface functions - Not to be used in new plugins
////////////////////////////////////////////////////////////////////////

int Win32SerialPort::numberOfFrames() {
    int n;

    frameLock_.lock();
    n = dataFrameQueue.size();
    frameLock_.unlock();

    return n;
}

FRAME* Win32SerialPort::firstFrame() {
    FRAME* f = NULL;

    frameLock_.lock();
    if (!dataFrameQueue.empty()) {
        f = dataFrameQueue.head();
    }
    frameLock_.unlock();

    return f;
}

int Win32SerialPort::removeFirstFrame() {

    frameLock_.lock();
    if (!dataFrameQueue.empty()) {
        delete dataFrameQueue.dequeue();
    }
    frameLock_.unlock();

    return 1;
}

void Win32SerialPort::setPpsSense(enum PpsSense ppsSense) {
  ppsSense_ = ppsSense;
}
