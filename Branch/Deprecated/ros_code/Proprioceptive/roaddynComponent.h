#ifndef ROADDYNCOMPONENT_H
#define ROADDYNCOMPONENT_H

#include <iostream>
#include <qthread>

#include "kernel/componentBase.h"
#include "XbowComponentConfig.h"

class XBOWCOMPONENT_API RoaddynComponent
        : public QThread
        , public ComponentBase
{
  Q_OBJECT

public:
	RoaddynComponent(QString name);
	~RoaddynComponent();

  virtual void stopActivity(); /*!< to stop the processing thread */
  virtual void startActivity(); /*!< to start the processing thread */
  virtual ComponentBase::COMPONENT_CONFIGURATION configureComponent(XmlComponentConfig config); 


public slots:
	void run();
  
private:
  road_time_t time_; 
  bool notEnd_;
};

#endif // ROADDYNCOMPONENT_H
