#include "icv2ros_core.h"
#include "OpenICV/Basis/icvStructureData.hxx"
#include "OpenICV/Basis/icvPrimitiveData.hxx"

icv2rosCore::icv2rosCore(ros::NodeHandle &nh)
{

    pub_number_ = nh.advertise<std_msgs::Int64>("/random/number",10);
    const std::string sub_name="tcp://127.0.0.1:2000";
    Register_Sub_Remote(sub_name,6970,1000,1000,1000);
    threadsList_[0] = boost::make_shared<boost::thread>(&icv2rosCore::Demo, this);  
    ros::spin();
}

icv2rosCore::~icv2rosCore() {
    for (int i = 0; i < MAX_THREAD_COUNT; i++) {
       	threadsList_[i]->join();
    }
    }

void icv2rosCore::Spin(){}

void icv2rosCore::Demo(){
    while(true){
         ROS_INFO("Into the demo");
         icv::data::icvInt64Data data_to_sub;
         std_msgs::Int64 data_to_pub;
         icvSubscribe_Remote("tcp://127.0.0.1:2000",&data_to_sub);
         cout<<data_to_sub.getvalue()<<endl;
         data_to_pub.data=data_to_sub.getvalue();
         pub_number_.publish(data_to_pub);
    }
}
 void icv2rosCore::Register_Sub_Remote(const string &sub_name, int port=6970,int sendpip=1000,int recvpip=1000,int timeout=1000) 
    {
        icv::core::ZMQ_Socket * zmq_sub;zmq_sub=new icv::core::ZMQ_Socket(context_zmq_uniq);
            zmq_sub=new icv::core::ZMQ_Socket();
            ICV_LOG_INFO<<"SUB   ini @"<< zmq_sub;
        
            zmq_sub->socket_ini(ZMQ_SUB, sub_name,sendpip,recvpip,timeout);

            zmq_subs->emplace(sub_name,zmq_sub);
       

    }
    void icv2rosCore::icvSubscribe_Remote(string sub,icv::core::icvDataObject* datatosend)
{
    // zmq_pollitem_t items[1];
    // items[0].socket = zmq_subs->at(sub);
    // items[0].events = ZMQ_POLLIN;
    //  zmq_poll(items, 1, 0);
        int size_;
        //cout<<"go to here subscribe"<<endl;
        // size_= zmq_recv(items[0].socket, buffer_receive, length, 0);
        
        string data;
        //if (items[0].revents& ZMQ_POLLIN) 
        zmq_subs->at(sub)->socket_recv(data);

        //ICV_LOG_INFO<<"RECEIVED DATA : "<<data;
        stringstream dstream(data);

        datatosend->Deserialize(dstream, 0); 



}
