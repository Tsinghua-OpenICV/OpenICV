#include "OpenICV/Core/icvNodeManager.h"

namespace icv { namespace core
{

        icvNodeManager::icvNodeManager(){};

        icvNodeManager::icvNodeManager(icv_map<std::string, icv_shared_ptr<icvNode>>* name_map) :node_name_map_(name_map)
        {
        long temp=icvTime::time_ns();
        try{
              
                if(name_map->size()>0)
                {
                        for (auto node_pair : *node_name_map_)
                        {
                                last_upd_time_of_nodes->emplace(node_pair.first,temp);
                                upd_period_of_nodes->emplace(node_pair.first,0);
                                thread_id_of_nodes->emplace(node_pair.first,icv_this_thread::get_id());

                        }
                }
        }
        catch(std::exception e){   ICV_LOG_INFO<<"manager problem"; }

        }
        icvNodeManager::~icvNodeManager(){};


        int icvNodeManager::getPriority(icv_thread *t)
        {
                sched_param sch;int polic;
                if(t->joinable())
                {
                pthread_getschedparam(t->native_handle(), &polic, &sch);
                return sch.sched_priority;
                
                }
                else return -1;

        };


        bool icvNodeManager::setPriority(icv_thread *t,int pri_level)
        {

                if(t->joinable())
                { 
                        sched_param sch;int polic;
                        pthread_getschedparam(t->native_handle(), &polic, &sch);
                        sch.sched_priority = 20;
                        if (pthread_setschedparam(t->native_handle(), SCHED_FIFO, &sch)) 
                        {
                                ICV_LOG_INFO<<"Failed to setschedparam: " << std::strerror(errno) ;
				return false;
                        }
			return true;
                }
                else   
		{
		ICV_LOG_INFO<<"Thread not joinable, not able to set priority" ;
		return false;
		}
        };

        void icvNodeManager::check_delayed_node()
        {
                for (auto node_pair : *node_name_map_)
                {
                        long delayedtime=upd_period_of_nodes->at(node_pair.first)/1000-node_pair.second->get_period();
                        ICV_LOG_INFO<<"node of "<<node_pair.first<<" , delayed time in micro seconds: "<<delayedtime<<" us";
                }




        }


        void icvNodeManager::rec_thread_id(string noden,icv_thread* t)
        {               
                thread_id_of_nodes->at(noden)=t->get_id();


        }
        void icvNodeManager::response_monitor(string nodename)
        {
                
                long temp_now_time=long(icvTime::time_ns());
                upd_period_of_nodes->at(nodename)=temp_now_time-last_upd_time_of_nodes->at(nodename);
                last_upd_time_of_nodes->at(nodename)=temp_now_time;
                //ICV_LOG_INFO<<"node of "<<nodename<<" ,time :"<<temp_now_time;

        }


}

}
