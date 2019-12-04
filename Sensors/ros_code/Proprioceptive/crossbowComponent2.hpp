#ifndef CROSSBOWCOMPONENT_HPP
#define CROSSBOWCOMPONENT_HPP

#include "kernel/componentBase.h"
#include "kernel/DbiteFile.h"
#include "driver/win32SerialPort.h"
#include "structure/structureXbow.h"
#include "VG700decoder.hpp"
#include "XbowComponentConfig.h"
#include "PacpusTools/ShMem.h"

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

private:
    VG700decoder vgDecoder_;

    char * currentDataFrame_; 
    int currentDataFrameLength_; 
    road_time_t currentRoadtime_; 
	XmlComponentConfig param;

    pacpus::DbiteFile crossbowhdFile_;
	QMap<int, ShMem *> shMem_;
    bool verbose_;
    double yaw_; 
    unsigned short prev_time_;
};

} // namespace pacpus

#endif // CROSSBOWCOMPONENT_HPP
