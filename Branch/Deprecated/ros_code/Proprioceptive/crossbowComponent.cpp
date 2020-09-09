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
	
}


void CrossbowComponent::startActivity()
{
	
    //char name[40];
	int id = 0;
	setMode(FILE_FLAG_OVERLAPPED); 
    //setMode(0);

    //openPort( xmlParameters().getProperty("port").toLatin1() );  
	   if (!openPort(xmlParameters().getProperty("port").toLatin1()))
  {
    LOG_INFO( "Failed to open the port " <<  xmlParameters().getProperty("port")) ;
    //LOG_INFO( "The  Component " << name() << " didn't start") ;
  }

	   else LOG_INFO( "open successfully the port: " << xmlParameters().getProperty("port")) ;

    configurePort(xmlParameters().getProperty("baudrate").toLong(), 
        xmlParameters().getProperty("bytesize").toUInt(),
        xmlParameters().getProperty("parity").toShort(),
        xmlParameters().getProperty("stopbits").toUInt() - 1); 


	senddata() ;
    verbose_ = xmlParameters().getProperty("verbose").toInt();
	if (verbose_) displayUI();
	//cout<<xmlParameters().getProperty("port")<<endl;
	
    crossbowhdFile_.open((ComponentBase::name() + ".dbt").toStdString(), WriteMode, CROSSBOW_VG700, sizeof(VG700dataframe));
    setActive( true );
	//id = CROSSBOW_VG700;
	//sprintf_s(name,"CROSSBOW_%d",id);
	//shMem_[id] = new ShMem(name, sizeof(VG700dataframe));
	outputVG700data_=getTypedOutput<VG700dataframe, CrossbowComponent>("VG700output");
	start(); 

}
void CrossbowComponent::addOutputs()
{

	addOutput<VG700dataframe, CrossbowComponent>("VG700output");


}
void CrossbowComponent::addInputs()
{
	



}

void CrossbowComponent::stopActivity()
{
	setActive( false );
	
	  if ( !closePort() )
    qDebug("Failed to close the port");
  else
    qDebug("The port is closed");
	
//LOG_INFO("closing DBT file");
if (verbose_)w->close();
 /* if (!wait(2000))
  {
    serialPort->terminate();
    qDebug("The Win32SerialPort thread blocks anormally, it has been killed !!");
  }

	*/
  


	if (crossbowhdFile_.isOpen())
	{
		LOG_INFO("closing DBT file");
		crossbowhdFile_.close();
	} else LOG_INFO("DBT file is not open");

	
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
	//cout<<" VG700 received data and data length is        "<<result.size()<<endl;
    // record data in a .dbt file
    for (size_t i = 0; i < result.size(); ++i) {
		if (crossbowhdFile_.isOpen())
           crossbowhdFile_.writeRecord(result[i].message_time, 0, (const char *) &(result[i]), sizeof(VG700dataframe));
		else 
			qDebug("DBT file is closed, data lost...");
		emit display1(result[i].accelX*9.8);
		emit display2(result[i].accelY*9.8);
		emit display3(result[i].accelZ*9.8);
		emit display4(result[i].yawRate*9.8);


		checkedSend(outputVG700data_,result[i]);

     /*   if (verbose_) 
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
		//shMem_[CROSSBOW_VG700]->write(&(result[i]),sizeof(VG700dataframe));
		*/
    }

    currentDataFrameLength_ = 0;
    delete[] currentDataFrame_;
    currentDataFrame_ = NULL;
    currentRoadtime_ = 0;
}


void CrossbowComponent::displayUI()
{
		// voir dans DbtPlyUserInterface§.cpp
	w = new QWidget();
	w->setWindowTitle(name());
	w->show();
	w->move(450,280);
	w->setFixedSize (170,180);
	w->setFont(QFont("Helvetica", 12, QFont::Normal)); 	
	prop = new QGroupBox ("VG700",w);
	prop->show();
	prop->setFixedSize(170,98);
	lab1 = new QLabel("ax ", prop);
	lab1->move( 5, 25);
	lab1->show();
	lab1->setFixedSize (80,30);
	qlcd1 = new QLCDNumber (prop);
	qlcd1->move( 5, 55);
	qlcd1->setFixedSize(80,30);
	qlcd1->setPalette(Qt::red);
	qlcd1->show();
	connect(this, SIGNAL(display1(double)) , qlcd1 , SLOT(display(double)));
	lab2 = new QLabel("ay  ", prop);
	lab2->move( 90, 25);
	lab2->show();
	lab2->setFixedSize (80,30); 
	qlcd2 = new QLCDNumber (prop);
	qlcd2->move( 90, 55);
	qlcd2->setFixedSize(80,30);
	qlcd2->show();
	qlcd2->setPalette(Qt::red);
	connect (this, SIGNAL(display2(double)) , qlcd2 , SLOT(display(double)));
	prop1 = new QGroupBox ("",w);
	prop1->show();
	prop1->move(0,100);
	prop1->setFixedSize(170,98);
	lab3 = new QLabel("az ", prop1);
	lab3->move( 5, 5);
	lab3->show();
	lab3->setFixedSize (80,30);
	qlcd3 = new QLCDNumber (prop1);
	qlcd3->move( 5, 35);
	qlcd3->setFixedSize(80,30);
	qlcd3->setPalette(Qt::red);
	qlcd3->show();
	connect(this, SIGNAL(display3(double)) , qlcd3 , SLOT(display(double)));
	lab4 = new QLabel("Yaw d/s ", prop1);
	lab4->move( 90, 5);
	lab4->show();
	lab4->setFixedSize (80,30);
	qlcd4 = new QLCDNumber (prop1);
	qlcd4->move( 90, 35);
	qlcd4->setFixedSize(80,30);
	qlcd4->show();
	qlcd4->setPalette(Qt::red);
	connect (this, SIGNAL(display4(double)) , qlcd4 , SLOT(display(double)));
	qlcd1->setDigitCount(5);
	qlcd2->setDigitCount(5);
	qlcd3->setDigitCount(5);
	qlcd4->setDigitCount(5);
}