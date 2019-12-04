#include "sensor/RoaddynComponent.h"
#include "kernel/ComponentFactory.h"
#include "C:\Program Files\National Instruments\NI-DAQ\DAQmx ANSI C Dev\include\NIDAQmx.h"
#include <fstream>


// Construct the factory
ComponentFactory<RoaddynComponent>* factory = new ComponentFactory<RoaddynComponent>("RoaddynComponent"); 

using namespace std;


RoaddynComponent::RoaddynComponent(QString name) : ComponentBase(name) 
{
	notEnd_ = true;
}


RoaddynComponent::~RoaddynComponent()
{

}


void RoaddynComponent::startActivity()
{
	notEnd_ = true;
	start();
}


void RoaddynComponent::stopActivity()
{
	notEnd_ = false;
	if (!wait(1000))
	{
		terminate();
		qDebug() << "The thread" << componentName << "seems blocked, it has been killed";
	}
}


ComponentBase::COMPONENT_CONFIGURATION RoaddynComponent::configureComponent(XmlComponentConfig config)
{

	return ComponentBase::CONFIGURED_OK;
}



/* TODO: 
- faire la mise à l'échelle pour convertir les tensions lues en signaux physiques (forces et moments)
- remplacer la méthode de polling utilisée pour l'acquisition : 
utliser les callbacks de NI pour lire les valeurs sur la carte dès qu'elles sont présentes 
- ajouter l'enregistrement en dbt
- ajouter les paramètres XML (choix des voies, de la carte, fréquence d'acquisition, etc.) 
*/
void RoaddynComponent::run()
{
	/*********************************************/
	// DAQmx Start Code
	/*********************************************/
	float64     data[6];
	TaskHandle  taskHandle=0;

	/*********************************************/
	// DAQmx Configure Code
	/*********************************************/

	DAQmxCreateTask("",&taskHandle);
	// Signaux connectés sur les voies analogiques ai0 à ai5
	DAQmxCreateAIVoltageChan(taskHandle,"Dev1/ai0:5","",DAQmx_Val_RSE ,-10.0,10.0,DAQmx_Val_Volts,NULL);
	DAQmxCfgSampClkTiming(taskHandle,"",500.0,DAQmx_Val_Rising,DAQmx_Val_ContSamps,6);

	DAQmxStartTask(taskHandle);

	int32 read = 0;
	ofstream resultFile;
	resultFile.open(componentName.toLatin1().data());
	while (notEnd_)
	{
		time_ = road_time(); 
		cout << time_ << endl; 
		DAQmxReadAnalogF64(taskHandle,1,10.0,DAQmx_Val_GroupByChannel,data,1000,&read,NULL);		
		cout << read << " ";
		if( read > 0 )
			resultFile << time_ << " " << 
			data[0] << " " <<
			data[1] << " " <<
			data[2] << " " <<
			data[3] << " " <<
			data[4] << " " <<
			data[5] << endl;
			//printf("Acquired %d samples, value = %f\n",read,data[0]);

		msleep(1);	// freq d'acquisition au max ~1kHz
	}
	resultFile.close();
}