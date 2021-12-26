#include "icvStdStreamSink.h"
#include "OpenICV/Core/icvDataObject.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Core/icvNodeManager.h"
#include <iostream>
namespace icv { namespace function
{    using namespace icv::data;


    ICV_REGISTER_FUNCTION(icvStdStreamSink)

    ICV_CONSTEXPR char KEY_OUTPUT_STREAM[] = "stream";

    icvStdStreamSink::icvStdStreamSink() : icvStdStreamSink(ICV_NULLPTR) {}
    icvStdStreamSink::icvStdStreamSink(icv_shared_ptr<icvMetaData> params)
        : icvFunction( params)
    {

    Register_Pub("Number");

    Register_Sub("Random number");
    //Register_Sub_Remote("tcp://127.0.0.1:2000");
    //Register_Sub_Remote("ipc://127.0.0.1:5555");
    
   
    }


    void icvStdStreamSink::Execute()
    {
       
       // count_++ ;
       //if (read_input_flag){
       
       //}
       // pub1=get_node()->GetMonitor()->GetAllPublisher().front();
      //ICV_LOG_INFO<<"Starting to execute";
        
        icv::data::icvInt64Data a,c;
        //cout<<"Now publishers have the number of "<<get_node()->GetMonitor()->GetAllPublisher().size()<<endl;

        // bool datatypecheck=false;

        // icv::core::icvDataObject* pub_buff= pub1->RequireDataObject();


        //     if(pub_buff) 
        //     {


               
        //             if(a->GetActualMemorySize()==pub_buff->GetActualMemorySize()&&a->GetActualMemorySize()>0){

        //             datatypecheck=true;
        //             }                    
                
        //         if(datatypecheck){ 
        //         pub_buff->CopyTo(a);
        //         a->empty=false; }
        //         else {ICV_LOG_WARN<<"SUBSCRIBE DATA TYPE NOT CORRESPONDED";}
                
        //     }
        //     else {  ICV_LOG_WARN<<"SUBSCRIBE DATA EMPTY"; }


        //icv::core::icvDataObject temp=*pub1->ReadDataObject();
        //a=static_cast<icv::data::icvInt64Data*>(pub1->ReadDataObject());
        //a=static_cast<icv::data::icvInt64Data*>(pub1->_data->DeepCopy());
        //a->empty=false;
        //icvSubscribe<icv::data::icvInt64Data>("Random number");
        //icvDataObject obj;
        //obj=sub1->RequireDataObject();
        ss.clear();
        icvSubscribe("Random number",&a);
     
        a.Serialize(ss,0);
        c.Deserialize(ss,0);
        c.empty=false;
        //icvSubscribe_Remote("tcp://127.0.0.1:2000",&a);
        //recorder::record(obj,string name);
        if(a.is_not_empty()){
        int b=a.getvalue();
        ICV_LOG_INFO<<"before : "<<b<<"  after : "<<3*b;
        }
        if(c.is_not_empty()){
        int b=c.getvalue();
        ICV_LOG_INFO<<"deserialized before : "<<b<<"  after : "<<3*b;
        icvPublish("Number",&c);

        }
    }
}}
