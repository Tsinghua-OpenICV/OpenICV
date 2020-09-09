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
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Core/icvZmq.h"
#include "OpenICV/Engine/icvConfigDefinitions.h"
#include <numeric>
#include <sstream>
using namespace icv::engine;
namespace icv { namespace core
{
using namespace std;

    // TODO: Add helpers to convert input/output pointers to related reference (like SafeDownCast?).
    //       This can be a function of icvDataObject
    class icvFunction : public icvObject
    {
    public:
    // Derivates should first pass params to this function through initializer list to initialize params from base class to derived classes
    icvFunction(icvNode* Node_ownner, icv_shared_ptr<const icvMetaData> info):
    node_owner_(Node_ownner)
    { if(info)
            {
                _information = *info; 
                if(_information.Contains("loop_time_micro"))
                    loop_time_microsec= _information.GetInteger("loop_time_micro") ;
                else loop_time_microsec=10000;

                if(_information.Contains(_keys::nodename))
                node_name_=_information.GetString(_keys::nodename);


                if (_information.Contains("buffer_size"))
                buffer_size_=_information.GetInteger("buffer_size");
                else buffer_size_=2048;

            }


    }

    icvFunction(icv_shared_ptr<const icvMetaData> info): 
    icvFunction(ICV_NULLPTR, info){}
    icvFunction()
    :icvFunction(ICV_NULLPTR, ICV_NULLPTR) { }

    virtual ~icvFunction() {


    }

    // DataObject pointers are mananged by input/output ports
    //virtual void Execute(icvDataObject** inData, icvDataObject** outData) = 0;
    virtual void Execute() = 0;
    //ICV_PROPERTY_GET(PublisherPtrvector, icvPublisher_, std::vector<icvPublisher*>&)
    // ICV_PROPERTY_GET(SubscriberPtrvector, icvSubscriber_, std::vector<icvSubscriber*>&)

    ICV_PROPERTY_GET(InputCount, _inputCount, uint8_t)
    ICV_PROPERTY_GET(OutputCount, _outputCount, uint8_t)

    // Hold loop of current node, take place only once for each Execute() call

    int get_loop_time_micro(){ return loop_time_microsec;};
    void RegisterSuspend(const icv_function<void(Duration64)>& func);

    void set_nodeowner(icvNode* node,std::string nodename)
    {
    node_owner_=node;

    };


    icvNode* get_node(){return node_owner_;}

    bool is_input_connected(string name)
    {


         return icvSubscriber_[name]->GetConnections().size()>0;


    };
    icvPublisher* Register_Pub(const string &pub_name){

            icvPublisher_.emplace(pub_name, new icvPublisher(this));
            icvPublisher_[pub_name]->setname(pub_name);
            pub_names_->push_back(pub_name);
            first_pub->emplace(icvPublisher_[pub_name],true);

            if(std::count(pub_names_->begin(), pub_names_->end(), pub_name)>1)
            ICV_THROW_MESSAGE("the publisher name exists, please change the name");
            count_pub++;


        return icvPublisher_[pub_name];


    };
    icvSubscriber* Register_Sub(const string &pub_name, bool trig=false) 
    {

            pubname_subnum_map->emplace( count_sub,pub_name);
            icvSubscriber_.emplace(pub_name,new icvSubscriber(this)) ;
            icvSubscriber_[pub_name]->setname(pub_name);
            sub_names_->push_back(pub_name);
            if(trig)icvSubscriber_[pub_name]->enable_Trigger();
            connections_->push_back(pair<string, string>(node_name_ + _keys::portSplitpoint + pub_name, pub_name));
            first_sub->emplace(icvSubscriber_[pub_name],true);
            if(std::count(connections_->begin(), connections_->end(), pair<string, string>(node_name_ + _keys::portSplitpoint + pub_name, pub_name))>1)
            ICV_THROW_MESSAGE("the subscriber name exists, please change the name");

            count_sub++;
            return icvSubscriber_[pub_name];

    };
    long vectorSum(vector<long> a)
    {
            long sum=0;
            for(int ix=0;ix<a.size();ix++){
            sum+=*(loop_seq.begin()+ix);           	
    } 

    return sum;
    }

    long get_loop_time(bool nanoseconds);

    vector<string> *get_pub_names(){return pub_names_;}
    vector<string> *get_sub_names(){return sub_names_;}





    void Register_Sub_Remote(const string &sub_name, int port=6970,int sendpip=1000,int recvpip=1000,int timeout=1000) 
    {
        ZMQ_Socket * zmq_sub;zmq_sub=new ZMQ_Socket(context_zmq_uniq);
        if(sub_name.find("inproc")==string::npos) 
        { 
            zmq_sub=new ZMQ_Socket();
            ICV_LOG_INFO<<"SUB   ini @"<< zmq_sub;
        
            zmq_sub->socket_ini(ZMQ_SUB, sub_name,sendpip,recvpip,timeout);

            zmq_subs->emplace(sub_name,zmq_sub);
        }
        else
        {

            inproc_sub_names.push_back(sub_name);

        }

    }


    void Register_Pub_Remote(const string &pub_name,int port=6974,int sendpip=1000,int recvpip=1000,int timeout=1000)
    {

        ZMQ_Socket * zmq_pub;zmq_pub=new ZMQ_Socket(context_zmq_uniq);
        if(pub_name.find("inproc:")==string::npos) 
        {
            zmq_pub=new ZMQ_Socket();
            zmq_pub->socket_ini(ZMQ_PUB, pub_name,sendpip,recvpip,timeout) ;
            zmq_pubs->emplace(pub_name,zmq_pub);
        }
        else
        {

            inproc_pub_names.push_back(pub_name);

        }
        
      
    }

    void Register_INPROC()
    {

        for(auto name_pub:inproc_pub_names)
        {

        ZMQ_Socket * zmq_pub=new ZMQ_Socket(context_zmq_uniq);
        zmq_pub->socket_ini(ZMQ_PUB, name_pub) ;
        zmq_pubs->emplace(name_pub,zmq_pub);

        }   
        
        for(auto name_sub:inproc_sub_names)
        {

        ZMQ_Socket * zmq_sub=new ZMQ_Socket(context_zmq_uniq);
        zmq_sub->socket_ini(ZMQ_SUB, name_sub) ;
        zmq_subs->emplace(name_sub,zmq_sub);

        }   
      
    }


  
    void icvPublish_Remote(string pub,icvDataObject* datatosend)
    {
        cout<<"go to here publish"<<endl;
            stringstream ss;
            datatosend->Serialize(ss,0);
            //ss<<"dada";
            zmq_pubs->at(pub)->socket_send(ss.str());
    }

    void icvSubscribe_Remote(string sub,icvDataObject* datatosend)
    {
       // zmq_pollitem_t items[1];
       // items[0].socket = zmq_subs->at(sub);
       // items[0].events = ZMQ_POLLIN;
      //  zmq_poll(items, 1, 0);
        int size_;
            cout<<"go to here subscribe"<<endl;
            // size_= zmq_recv(items[0].socket, buffer_receive, length, 0);
           
            string data;
            //if (items[0].revents& ZMQ_POLLIN) 
            zmq_subs->at(sub)->socket_recv(data);

            //ICV_LOG_INFO<<"RECEIVED DATA : "<<data;
            stringstream dstream(data);

            datatosend->Deserialize(dstream, 0); 


   
    }





    void addconnections(vector<pair<string, string>>* connect_all)

    {
         connect_all->insert(connect_all->end(),connections_->begin(),connections_->end());


    }

    void icvSubscribe(string sub,icvDataObject* data)
    {



                if(icvSubscriber_.at(sub)->isconnected())

                {


                icvDataObject* sub_buff= icvSubscriber_.at(sub)->RequireDataObject();


                if(sub_buff) 
                {


                    if(first_sub->at(icvSubscriber_.at(sub)))
                    {
                        if(data->GetActualMemorySize()==sub_buff->GetActualMemorySize()&&data->GetActualMemorySize()>0)

                        datatypecheck=true;
                        first_sub->at(icvSubscriber_.at(sub))=false;
                        
                    }
                    if(datatypecheck){ 
                    sub_buff->CopyTo(data);
                    data->empty=false; }
                    else {ICV_LOG_WARN<<"SUBSCRIBE DATA TYPE NOT CORRESPONDED";}
                    
                }
                else {  ICV_LOG_WARN<<"SUBSCRIBE DATA EMPTY"; }
  
                            



            }
    }
    template <typename T>  T* icvSubscribe(string sub)
    {



                if(icvSubscriber_.at(sub)->isconnected())

                {


                icvDataObject* sub_buff= icvSubscriber_.at(sub)->RequireDataObject();


                if(sub_buff) 
                {

                    if(first_sub->at(icvSubscriber_.at(sub)))
                    {
                        datatypecheck=CheckDataType<T>(icvSubscriber_.at(sub));
                        first_sub->at(icvSubscriber_.at(sub))=false;
                        
                    }
                    if(datatypecheck) 
                    return static_cast<T*>(sub_buff); 
                    else ICV_LOG_FATAL<<"SUBSCRIBE DATA TYPE NOT CORRESPONDED";
                    
                }
                else{


                    
                T* temp_=new T();
                return temp_; 
                } 
                            



            }
    }


    void icvPublish(string pub,icvDataObject* datatosend)
    {

        if(find(pub_names_->begin(), pub_names_->end(), pub)==pub_names_->end()) Register_Pub(pub);

        icvPublisher_.at(pub)->set_send_data(datatosend);
        icvPublisher_.at(pub)->Send_Out();



    }

    bool has_Triggerinput()
    {
            int count_trig=0;
            for(auto substemp:icvSubscriber_) 
            {
            if(substemp.second->is_Triggered())count_trig++;
            }
            if (count_trig<1) return false;
            else if (count_trig>1) {return false;ICV_LOG_WARN<<"More than one Trigger in one function";}
            else return true;
		 

    };


    icvSubscriber* GetInputPort(string port)
    {
            if (find(sub_names_->begin(), sub_names_->end(), port)==sub_names_->end())
            icvSubscriber_.emplace(port,new icvSubscriber(this));


            return icvSubscriber_[port];
    };

    icvPublisher* GetOutputPort(string port)
    {
            if (find(pub_names_->begin(), pub_names_->end(), port)==pub_names_->end())
            icvPublisher_.emplace(port,new icvPublisher(this));


            return icvPublisher_[port];
    };


    void set_zmq_context_uniq(void* context){context_zmq_uniq= context;                 
};

    icv_map< int,std::string>* get_pubsub_map(){ return pubname_subnum_map; };




    icvDataObject** getIndataptr(){return inData;};
    icvDataObject** getOutdataptr(){return outData;};

    icvDataObject* read_Input_Ptr(int i){return inData[i];};
    icvDataObject* send_Output_Ptr(int i){return outData[i];};







    protected:
    icvMetaData _information;
    icv_function<void(Duration64)> _sleepFunc;

    private:
    uint8_t _inputCount, _outputCount; 
    icvDataObject** inData;
    icvDataObject** outData;
    std::vector<bool> _first_send;//first time to send data
    icv_map<string,icvPublisher*> icvPublisher_;
    icv_map<string,icvSubscriber*> icvSubscriber_;
    icv_map< string,icvDataObject::Ptr>* publishdata_= new icv_map< string,icvDataObject::Ptr>();
    icv_map<icvPublisher*,bool>* first_pub= new icv_map<icvPublisher*,bool>();
    icv_map<icvSubscriber*,bool>* first_sub= new icv_map< icvSubscriber*,bool>();

    icv_mutex dataoutlock;
    int count_pub=0,count_sub=0;
    icv_map< int,std::string>* pubname_subnum_map= new icv_map< int,std::string>();
    int loop_time_microsec;
    icvNode* node_owner_;
    vector<pair<string, string>>* connections_= new vector<pair<string, string>>();
    vector<string>*  pub_names_= new vector<string>();
    vector<string>*  sub_names_= new vector<string>();

    std::string node_name_;
    vector<time_t> time_seq;
    vector<long>loop_seq;
    vector<string> inproc_pub_names;
    vector<string> inproc_sub_names;

    time_t time_aver;
    int count_time=0;
    bool datatypecheck;

    void* context_zmq_uniq=ICV_NULLPTR;
    // vector<void *> sendingSocketList;
    int buffer_size_=2048;
    icv_map<string,ZMQ_Socket*>* zmq_subs=new icv_map<string,ZMQ_Socket*>();
    icv_map<string,ZMQ_Socket*>* zmq_pubs=new icv_map<string,ZMQ_Socket*>();

    };

    // TODO: Add helper function RequireProperty<Type>(const std::string& name) to help derivates to initialize properties.
    }}

    #endif // icvFunction_h
