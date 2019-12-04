#ifndef icvCvVideoCaptureSource_h
#define icvCvVideoCaptureSource_h

#include "OpenICV/Core/icvFunction.h"
#include <opencv2/videoio.hpp>
#include "OpenICV/structure/icvCvMatData.hxx"

namespace icv
{
    namespace opencv
    {
        using namespace icv::core;

        class icvCvVideoCaptureSource : public icvFunction
        {
        public:
            icvCvVideoCaptureSource();
            icvCvVideoCaptureSource(icv_shared_ptr<const icvMetaData> info);
            
         

            virtual void Execute(std::vector<icvNodeInput*>& inputPorts, std::vector<icvNodeOutput*>& outputPorts) ICV_OVERRIDE;

        private:
            cv::VideoCapture _cap;cv::Mat mFrame;
            icv::data::icvCvMatData *tempdata; 

        };
    }
}

#endif // icvCvVideoCaptureSource_h
