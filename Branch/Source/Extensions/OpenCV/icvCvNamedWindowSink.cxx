#include "OpenICV/Extensions/OpenCV/icvCvNamedWindowSink.h"
#include "OpenICV/structure/icvCvMatData.hxx"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Core/icvSubscriber.h"
using namespace cv;
using namespace std;

namespace icv
{
    using namespace data;
    namespace opencv
    {
        ICV_REGISTER_FUNCTION(icvCvNamedWindowSink)

        unsigned int icvCvNamedWindowSink::_name_counter = 0;
        ICV_CONSTEXPR const char KEY_WINDOW_NAME[] = "name";

        icvCvNamedWindowSink::icvCvNamedWindowSink() : icvCvNamedWindowSink(ICV_NULLPTR) {}
        icvCvNamedWindowSink::icvCvNamedWindowSink(icv_shared_ptr<const icvMetaData> info)
            : icvFunction(info)
        {
            if (_information.Contains(KEY_WINDOW_NAME))
                _name = _information.GetString(KEY_WINDOW_NAME);
            else
                _name = "anonymous" + to_string(_name_counter++);
            sub1=Register_Sub("cam_image");
        }


        void icvCvNamedWindowSink::Execute()
    {

            if (!_created)
            {
                // Window should be created in the same thread as imshow
                namedWindow(_name);
                _created = true;
            }
    
            cv::Mat& image = icvSubscribe<icvCvMatData>("cam_image")->getvalue();
            imshow(_name, image);

            waitKey(1);
        }
    }
}