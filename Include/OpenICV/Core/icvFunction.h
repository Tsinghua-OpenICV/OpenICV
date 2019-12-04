#ifndef icvFunction_h
#define icvFunction_h

#define ICV_CONFIG_POINTERS
#define ICV_CONFIG_FUNCTION
#include "OpenICV/Core/icvConfig.h"
#undef ICV_CONFIG_POINTERS
#undef ICV_CONFIG_FUNCTION

#include <vector>
#include "OpenICV/Core/icvObject.h"
#include "OpenICV/Core/icvMacros.h"
#include "OpenICV/Core/icvMetaData.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
namespace icv { namespace core
{
    class icvDataObject;
    class icvMetaData;

    // TODO: Add helpers to convert input/output pointers to related reference (like SafeDownCast?).
    //       This can be a function of icvDataObject
    class icvFunction : public icvObject
    {
    public:
        // Derivates should first pass params to this function through initializer list to initialize params from base class to derived classes
        icvFunction(uint8_t inputCount, uint8_t outputCount, icv_shared_ptr<const icvMetaData> info)
            :_inputCount(inputCount), _outputCount(outputCount)
        { if(info) _information = *info; }
         icvFunction(icv_shared_ptr<const icvMetaData> info)
        { if(info) _information = *info; }
        icvFunction(uint8_t inputCount, uint8_t outputCount)
            :icvFunction(inputCount, outputCount, ICV_NULLPTR) { }

        virtual ~icvFunction() { }

        // DataObject pointers are mananged by input/output ports
        //virtual void Execute(icvDataObject** inData, icvDataObject** outData) = 0;
        virtual void Execute(std::vector<icvNodeInput*>& inputPorts, std::vector<icvNodeOutput*>& outputPorts) = 0;


        ICV_PROPERTY_GET(InputCount, _inputCount, uint8_t)
        ICV_PROPERTY_GET(OutputCount, _outputCount, uint8_t)

        // Hold loop of current node, take place only once for each Execute() call
        void RegisterSuspend(const icv_function<void(Duration64)>& func);

        // Input/Output port pointers are managed by nodes
         void ConfigurateInput(std::vector<icvNodeInput*>& inputPorts)
         {  
             inData = new icvDataObject*[inputPorts.size()];

            inputPorts_.assign(inputPorts.begin(),inputPorts.end());
            _inputCount=inputPorts.size();

            };
         void ConfigurateOutput(std::vector<icvNodeOutput*>& outputPorts)
          {

            outData = new icvDataObject*[outputPorts.size()];
            outputPorts_.assign(outputPorts.begin(),outputPorts.end());
            _outputCount=outputPorts.size();
            
            _first_send.resize(outputPorts.size());
            for(int i=0;i<outputPorts.size();i++)_first_send.at(i)=true;
          };
   
        bool is_input_connected(int port)
        {
            
            
            return inputPorts_[port]->GetConnections().size()>0;
        
        
        };
    
        void buff_Input(std::vector<icvNodeInput*>& inputPorts)
        {      
             for (int i = 0; i < inputPorts.size(); i++)
            {
                if(inputPorts[i]->GetConnections().size()>0)

                {

                    inData[i] = inputPorts[i]->RequireDataObject();
                }

                else ICV_LOG_INFO<<"InputPort "<<i<<" not connected";
            }
        };
        void buff_Output(std::vector<icvNodeOutput*>& outputPorts)
        { 
            for (int i = 0; i < outputPorts.size(); i++)
            {
                outData[i] = outputPorts[i]->WriteDataObject();
            }
            
        };
       icvDataObject** getIndataptr(){return inData;}
       icvDataObject** getOutdataptr(){return outData;}

       icvDataObject* read_Input_Ptr(int i){return inData[i];}
       icvDataObject* send_Output_Ptr(int i){return outData[i];}
        void Send_Out(icvDataObject* data_to_send,int port)
       {
        //send_Output<icv::data::icvInt64Data >(port);

		dataoutlock.lock();
        if(_first_send[port])
        {
            outputPorts_[port]->SetDataObject(data_to_send);
            _first_send[port]=false;
        }
        buff_Output(outputPorts_);
       // data_to_send->SetSourceTime(SyncClock::time_us());
		data_to_send->CopyTo(outData[port]);
        outputPorts_[port]->flag_update();
        dataoutlock.unlock();
       };
      template <typename T>   icvDataObject* Read_In(icvDataObject* data_to_read,int port)
       {

		//dataoutlock.lock();
		//read_Input<icv::data::icvInt64Data >(port).CopyTo(data_to_read);
        data_to_read=&read_Input<T>(port);
        //dataoutlock.unlock();
        //outputPorts_[port]->flag_update();
        return data_to_read;


       };
       // port i has data come in?
      bool is_not_empty(int i){
         // ICV_LOG_INFO<<"check EMPTY";  
          if(inData[i])
          {
              if(inData[i]->Is_Reserved() ) 
                { 
                    return true;
                }

          else 

                { 
                return false;
                }
          }
           else 

                {
                    return false; 
                } 
                
          }
    //template <typename T>  T& send_Output(int i) { return outData[i]->As<T>(); }

    template <typename T>  T& read_Input(int i) 
    {  
        if(is_not_empty(i)) 
        {
            // ICV_LOG_INFO<<"input not empty";
            return inData[i]->As<T>(); 
        }
        else{
            // ICV_LOG_INFO<<"input empty";
            T* temp_=new T(); 
            return *temp_; 
        } 
    }



    protected:
        icvMetaData _information;
        icv_function<void(Duration64)> _sleepFunc;

    private:
        uint8_t _inputCount, _outputCount; 
        icvDataObject** inData;
        icvDataObject** outData;
        std::vector<bool> _first_send;//first time to send data
        std::vector<icvNodeOutput*> outputPorts_;
        std::vector<icvNodeInput*> inputPorts_;
        icv_mutex dataoutlock;

    };

    // TODO: Add helper function RequireProperty<Type>(const std::string& name) to help derivates to initialize properties.
}}

#endif // icvFunction_h
