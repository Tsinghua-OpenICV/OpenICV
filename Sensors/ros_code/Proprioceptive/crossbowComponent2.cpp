/**
to see: send T and C command to the Xbow unit
**/

#include "CrossbowComponent.hpp"

#include <iostream>

#include "kernel/ComponentFactory.h"
#include "kernel/DbiteFileTypes.h"

using namespace pacpus;
using namespace std;

/// Construct the factory
ComponentFactory<CrossbowComponent> sFactory("CrossbowComponent"); 

CrossbowComponent::CrossbowComponent(QString name)
    : Win32SerialPort(name)
    , ComponentBase(name) 
{
    verbose_ = false; 
    yaw_ = 0.0;
    prev_time_ = 0;

    if (!connect(this,SIGNAL(newDataAvailable(int)), this, SLOT(decodeFrame(int)))) {
        qWarning("Failed to connect SIGNAL(newDataAvailable(int)) with SLOT(decodeFrame(int)\n" ); 
    }
	else  qWarning("succeed to connect SIGNAL(newDataAvailable(int)) with SLOT(decodeFrame(int)\n" );
}


CrossbowComponent::~CrossbowComponent()
{
	foreach (ShMem * value, shMem_)
	{
		delete value;
    }
}


void CrossbowComponent::startActivity()
{
    char name[40];
	int id = 0;
	setMode(FILE_FLAG_OVERLAPPED); 
    //setMode(0);

   // openPort( param.getProperty("port").toLatin1() );  
	openPort(xmlParameters().getProperty("port").toLatin1());
    /*configurePort(param.getProperty("baudrate").toLong(), 
        param.getProperty("bytesize").toUInt(),
        param.getProperty("parity").toShort(),
        param.getProperty("stopbits").toUInt() - 1); 
    verbose_ = param.getProperty("verbose").toInt();
	*/
	   configurePort(xmlParameters().getProperty("baudrate").toLong(), 
        xmlParameters().getProperty("bytesize").toUInt(),
        xmlParameters().getProperty("parity").toShort(),
        xmlParameters().getProperty("stopbits").toUInt() - 1); 
    verbose_ = xmlParameters().getProperty("verbose").toInt();


    crossbowhdFile_.open((ComponentBase::name() + ".dbt").toStdString(), WriteMode, CROSSBOW_VG700, sizeof(VG700dataframe));
    Win32SerialPort::THREAD_ALIVE = TRUE; 
	id = CROSSBOW_VG700;
	sprintf_s(name,"CROSSBOW_%d",id);
	shMem_[id] = new ShMem(name, sizeof(VG700dataframe));
	start(); 
}


void CrossbowComponent::stopActivity()
{
	qDebug("stopping Crossbow component...");
    Win32SerialPort::THREAD_ALIVE = FALSE; 
    if (!wait(2000)) {
        terminate(); 
        qDebug("The Win32SerialPort thread blocks anormally, it has been killed !!");
    }
    if ( !closePort() ) {
        qDebug("Failed to close the port"); 
    } else {
        qDebug("The port is closed"); 
    }
	if (crossbowhdFile_.isOpen())
	{
		qDebug("closing DBT file");
		crossbowhdFile_.close();
	} else qDebug("DBT file is not open");

	
}

ComponentBase::COMPONENT_CONFIGURATION CrossbowComponent::configureComponent(XmlComponentConfig config)

{

    return ComponentBase::CONFIGURED_OK;
}

void CrossbowComponent::decodeFrame(int)
{
    // get the frame and remove it in the list
    FRAME * currentFrame = firstFrame(); 

    currentDataFrame_ = new char[currentFrame->length]; 
    memcpy(currentDataFrame_, currentFrame->data, currentFrame->length); 
    currentDataFrameLength_ = currentFrame->length; 
    currentRoadtime_ = currentFrame->t; 

    removeFirstFrame(); 

    // decode data 
    vector<VG700dataframe> result = vgDecoder_.decode(currentDataFrame_, currentDataFrameLength_,currentRoadtime_);

    // record data in a .dbt file
    for (size_t i = 0; i < result.size(); ++i) {
		if (crossbowhdFile_.isOpen())
           crossbowhdFile_.writeRecord(result[i].message_time, 0, (const char *) &(result[i]), sizeof(VG700dataframe));
		else 
			qDebug("DBT file is closed, data lost...");

        //if (verbose_) 
		{
            // Computation of data output period from the time flag
            unsigned short usperiod = prev_time_ - result[i].embedded_time;
            double period = 0.79e-6 * usperiod;
            // Just to estimate the drift, yaw computation by integration of the yaw_rate
            yaw_ += period * result[i].yawRate;

            prev_time_ = result[i].embedded_time;


            // Display decoded data
            cout.setf(ios::fixed);
            cout.precision(3);
            cout.width(8);
            cout
                << "r=" <<result[i].rollAngle
                <<" p=" <<result[i].pitchAngle
                //<<" rr="<<result[i].rollRate
                //<<" pr="<<result[i].pitchRate
                //<<" yr="<<result[i].yawRate
                <<" ax="<<result[i].accelX
                <<" ay="<<result[i].accelY
                //<<" az="<<result[i].accelZ
                //<<" t="<<result[i].temp
                <<" yaw=" << yaw_
                <<" f=" << 1. / period
                <<endl;
        }
		shMem_[CROSSBOW_VG700]->write(&(result[i]),sizeof(VG700dataframe));
    }

    currentDataFrameLength_ = 0;
    delete[] currentDataFrame_;
    currentDataFrame_ = NULL;
    currentRoadtime_ = 0;
}


