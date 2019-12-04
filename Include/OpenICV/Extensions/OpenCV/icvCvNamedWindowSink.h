#ifndef icvCvNamedWindowSink_h
#define icvCvNamedWindowSink_h

#include "OpenICV/Core/icvFunction.h"
#include <opencv2/highgui.hpp>

namespace icv
{
    namespace opencv
    {
        using namespace icv::core;

        class icvCvNamedWindowSink : public icvFunction
        {
        public:
            icvCvNamedWindowSink();
            icvCvNamedWindowSink(icv_shared_ptr<const icvMetaData> info);


            virtual void Execute(std::vector<icvNodeInput*>& inputPorts, std::vector<icvNodeOutput*>& outputPorts) ICV_OVERRIDE;

        private:
            std::string _name;
            bool _created = false;

            static unsigned int _name_counter;
        };
    }
}

#endif // icvCvNamedWindowSink_h
