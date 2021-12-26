#ifndef icvNodeMonitor_h
#define icvNodeMonitor_h

#define ICV_CONFIG_THREAD
#include "OpenICV/Core/icvObject.h"
#undef ICV_CONFIG_THREAD
#include <pthread.h>
#include "OpenICV/Core/icvMacros.h"
#include "OpenICV/Core/icvNode.h"
#include "OpenICV/Core/icvTime.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
namespace icv { namespace core
{
    class icvNode;
    class icvSubscriberInterface;
    class icvPublisherInterface;
    
    class icv_thread_guard
{
        icv_thread &t;
        public :
        explicit icv_thread_guard(icv_thread& _t) :
            t(_t){}

        ~icv_thread_guard()
        {
            if (t.joinable())
                t.detach();
        }

        icv_thread_guard (const icv_thread_guard&) = delete;
        icv_thread_guard& operator=(const icv_thread_guard&) = delete;
};
class icvNodeManager : public icvObject
{
    public:

        icvNodeManager();
        icvNodeManager(icv_map<std::string, icv_shared_ptr<icvNode>>* name_map=ICV_NULLPTR);

         ~icvNodeManager();
        int getPriority(icv_thread* t);
        bool setPriority(icv_thread *t,int pri_level);
        void response_monitor(string nodename);
        void check_delayed_node();
        void rec_thread_id(string name,icv_thread* t);
        vector<icvPublisherInterface*> GetAllPublisher(){return AllPublisher;}

    private:
        icv_map<string,long>* last_upd_time_of_nodes=new icv_map<string,long>();
        icv_map<string,long>* upd_period_of_nodes=new  icv_map<string,long>();
        icv_map<string,icv_thread::id>* thread_id_of_nodes=new  icv_map<string,icv_thread::id>();
        icv_map<std::string, icv_shared_ptr<icvNode>>* node_name_map_;
        std::vector<icvPublisherInterface*> AllPublisher;

};
}}

#endif // icvNodeMonitor_h
