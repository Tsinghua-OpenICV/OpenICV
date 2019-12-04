/*********************************************************************
// created:    2007/04/12 - 16:30

  //
  // author:     Elie Al Alam & Gerald Dherbomez
  // 
  // version:    $Id: DbtPlyAbsManager.cpp 875 2010-07-23 16:07:47Z gdherbom $
  //
  // purpose:    Dbite Player ABS file Manager implementation
*********************************************************************/

#include "kernel/ComponentManager.h"
#include <qapplication.h>
#include "DbitePlayer/DbtPlyAbsManager.h"


static ComponentFactory<DbtPlyAbsManager>* factory = new ComponentFactory<DbtPlyAbsManager>("DbtPlyAbsManager"); 




DbtPlyAbsManager::DbtPlyAbsManager(QString name):DbtPlyFileManager (name)
{
#if WIN32
  shMem_ = new ShMem("ABS",sizeof(donnees_abs_zy));
#endif
}



DbtPlyAbsManager::~DbtPlyAbsManager()
{ 
#if WIN32
	delete shMem_;
#endif
}



void DbtPlyAbsManager::processData(road_time_t t, road_timerange_t tr , void *buf)
{
  if (buf==NULL)
  {
    emit displayAbsVAvG(0);
  }
  else
  {
    val = (data_abs_original *)(buf);
    printf("v=%f\n",val->vitesse_ar_d);
    
    dataToTransmit_.dArD = val->dist_parc_ar_d ;
    dataToTransmit_.dArG = val->dist_parc_ar_g ;
    dataToTransmit_.speed=(val->vitesse_ar_d + val->vitesse_ar_g)/2;//moyenne des roues arrieres
#if WIN32
    shMem_->write(&dataToTransmit_, sizeof(donnees_abs_zy));
#endif
    
    emit displayAbsVAvG(val->vitesse_av_g);
    emit displayAbsVAvD(val->vitesse_av_d);
    emit displayAbsVArG(val->vitesse_ar_g);
    emit displayAbsVArD(val->vitesse_ar_d);
  }
}



void DbtPlyAbsManager::displayUI()
{
  w = new QWidget();
  w->setWindowTitle(componentName);
  w->show();
  w->setFixedSize (170,110);
  
  speed = new QGroupBox ("ABS->Speed",w);
  speed->show();
  speed->setFixedSize(170,105);
  
  labVAvG = new QLabel("Av Gauche ", speed);
  labVAvG->move( 5, 15);
  labVAvG->show();
  labVAvG->setFixedSize (80,20);
  lcdVAvG = new QLCDNumber (speed);
  lcdVAvG->move( 5, 35);
  lcdVAvG->setFixedSize(50,20);
  lcdVAvG->show();
  connect (this, SIGNAL(displayAbsVAvG(double)) , lcdVAvG , SLOT(display(double)));
  
  labVAvD = new QLabel("Av Droite ", speed);
  labVAvD->move( 90, 15);
  labVAvD->show();
  labVAvD->setFixedSize (80,27);
  
  lcdVAvD = new QLCDNumber (speed);
  lcdVAvD->move( 90, 35);
  lcdVAvD->setFixedSize(50,20);
  lcdVAvD->show();
  connect (this, SIGNAL(displayAbsVAvD(double)) , lcdVAvD , SLOT(display(double)));
  
  labVArG = new QLabel("Ar Gauche ", speed);
  labVArG->move( 5, 60);
  labVArG->show();
  labVArG->setFixedSize (80,20);
  lcdVArG = new QLCDNumber (speed);
  lcdVArG->move( 5, 80);
  lcdVArG->setFixedSize(50,20);
  lcdVArG->show();
  connect (this, SIGNAL(displayAbsVArG(double)) , lcdVArG , SLOT(display(double)));
  
  labVArD = new QLabel("Ar Droite ", speed);
  labVArD->move( 90, 60);
  labVArD->show();
  labVArD->setFixedSize (80,20);
  lcdVArD = new QLCDNumber (speed);
  lcdVArD->move( 90, 80);
  lcdVArD->setFixedSize(50,20);
  lcdVArD->show();
  connect (this, SIGNAL(displayAbsVArD(double)) , lcdVArD , SLOT(display(double))); 
}

