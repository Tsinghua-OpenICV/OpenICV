/*********************************************************************
// created:    2007/04/12 - 16:30

//
// author:     Elie Al Alam & Gerald Dherbomez
// 
// version:    $Id: DbtPlyGyrokvhManager.cpp 875 2010-07-23 16:07:47Z gdherbom $
//
// purpose:    Dbite Player Gyrokvh Manager implementation
*********************************************************************/

#include <QApplication>
#include "DbitePlayer/DbtPlyGyrokvhManager.h"

// Construction de la fabrique de composant DbtPlyTrigger
ComponentFactory<DbtPlyGyrokvhManager>* factory = new ComponentFactory<DbtPlyGyrokvhManager>("DbtPlyGyrokvhManager"); 


DbtPlyGyrokvhManager::DbtPlyGyrokvhManager(QString name):DbtPlyFileManager (name)
{
  ptrser = NULL;
	ptr= ComponentManager::create();//added
#ifdef WIN32
  shMem_ = new ShMem("GYROKVH",sizeof(donnees_gyro));
#endif
}

DbtPlyGyrokvhManager::~DbtPlyGyrokvhManager(){ 
#if WIN32
	delete shMem_;
#endif
}


void DbtPlyGyrokvhManager::processData(road_time_t t, road_timerange_t tr , void *buf){

  if (buf==NULL)
    return;
  else
  {
    val = (*(data_gyroKVH*)buf);
    if (ptrser)
      ptrser->cs->setgyrokvh(val,t);

    dataToTransmit_.psi_s = val.psi_s ;
#ifdef WIN32
    shMem_->write(&dataToTransmit_, sizeof(data_gyroKVH));
#endif
    emit displayGyroPsiS(val.psi_s);
    emit displayGyroPPsiS(val.p_psi_s);
  }
}



ComponentBase::COMPONENT_CONFIGURATION DbtPlyGyrokvhManager::configureComponent(XmlComponentConfig config)
{
  DbtPlyFileManager::configureComponent(config);
  ptrser=(SensorTcpServer*)ptr->getComponent("server");//added
  if (!ptrser)
    qDebug() << componentName << " server is not present, data won't be transfered by the network";
  return ComponentBase::CONFIGURED_OK;
}



void DbtPlyGyrokvhManager::startActivity()
{ 
  DbtPlyFileManager::startActivity();

  // user interface 
  if (ui_)
    displayUI(); 

  
}



void DbtPlyGyrokvhManager::stopActivity()
{
  DbtPlyFileManager::stopActivity();
}



void DbtPlyGyrokvhManager::displayUI()
{

  w = new QWidget();
  w->setWindowTitle(componentName); 
  w->show();
  w->setFixedSize(130,100);

  gyroData = new QGroupBox ( "GyroKVH Data" );
  gyroData->show();
  gyroData->setFixedSize(120,90);

  labPsiS = new QLabel("Psi_S",gyroData);
  labPsiS->show();

  lcdPsiS = new QLCDNumber(gyroData);
  lcdPsiS->show();
  connect (this, SIGNAL(displayGyroPsiS(double)) , lcdPsiS , SLOT(display(double)));

  labPPsiS = new QLabel("P_Psi_S", gyroData);
  labPPsiS->show();

  lcdPPsiS = new QLCDNumber(gyroData);
  lcdPPsiS->show();
  connect (this, SIGNAL(displayGyroPPsiS(double)) , lcdPPsiS , SLOT(display(double)));

}
