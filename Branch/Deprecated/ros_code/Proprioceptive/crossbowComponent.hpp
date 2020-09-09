#ifndef CROSSBOWCOMPONENT_HPP
#define CROSSBOWCOMPONENT_HPP

#include "kernel/componentBase.h"
#include "kernel/DbiteFile.h"
#include "driver/win32SerialPort.h"
#include "structure/structureXbow.h"
#include "VG700decoder.hpp"
#include "XbowComponentConfig.h"
#include "PacpusTools/ShMem.h"
#include <QLCDNumber>
#include <QGroupBox>
#include <QLabel>
#include <QLayout>
namespace pacpus {

	class ShMem;


class XBOWCOMPONENT_API CrossbowComponent
        : public Win32SerialPort
        , public ComponentBase
{
    Q_OBJECT

public:
    CrossbowComponent (QString name);
    ~CrossbowComponent ();

    virtual void stopActivity(); /*!< to stop the processing thread */
    virtual void startActivity(); /*!< to start the processing thread */
    virtual ComponentBase::COMPONENT_CONFIGURATION configureComponent(XmlComponentConfig config); 

public Q_SLOTS:
    void decodeFrame(int i); 
		Q_SIGNALS:
	void display1(double);
    void display2(double);
	void display3(double);
    void display4(double);

private:
	void displayUI();
	   QWidget *w;
    QLabel* lab1,*lab2,*lab3,*lab4;
    QLCDNumber *qlcd1,*qlcd2,*qlcd3,*qlcd4;
    QGroupBox *prop,*prop1;
	 virtual void addInputs();
    virtual void addOutputs();
	QString portName_;
	void setPortCOM(const char * port);
     OutputInterface<VG700dataframe, CrossbowComponent>* outputVG700data_;
    VG700decoder vgDecoder_;
	Win32SerialPort *serialPort;
    char * currentDataFrame_; 
    int currentDataFrameLength_; 
    road_time_t currentRoadtime_; 

    pacpus::DbiteFile crossbowhdFile_;
	//QMap<int, ShMem *> shMem_;
    bool verbose_;
    double yaw_; 
    unsigned short prev_time_;
};

} // namespace pacpus

#endif // CROSSBOWCOMPONENT_HPP
