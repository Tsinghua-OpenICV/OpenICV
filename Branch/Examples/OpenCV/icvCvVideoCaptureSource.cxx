#include "icvCvVideoCaptureSource.h"
#include "OpenICV/structure/icvCvMatData.hxx"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Core/icvSubscriber.h"
using namespace cv;
using namespace std;

namespace icv
{
    namespace opencv
    {
        ICV_REGISTER_FUNCTION(icvCvVideoCaptureSource)

        ICV_CONSTEXPR char KEY_CAMERA_ID[] = "camera_id";
        ICV_CONSTEXPR char KEY_WIDTH[] = "width";
        ICV_CONSTEXPR char KEY_HEIGHT[] = "height";

        icvCvVideoCaptureSource::icvCvVideoCaptureSource() : icvCvVideoCaptureSource(ICV_NULLPTR) {}
        icvCvVideoCaptureSource::icvCvVideoCaptureSource(icv_shared_ptr<const icvMetaData> info)
            : icvFunction(info)
        {
            if (_information.Contains(KEY_CAMERA_ID))
                _cap = VideoCapture(_information.GetInteger(KEY_CAMERA_ID));
            else
                _cap = VideoCapture(0);

            if (!_cap.isOpened())  // check if succeeded to connect to the camera 
                throw "Camera opened failed!"; 
            
            if (_information.Contains(KEY_WIDTH))
                _cap.set(CAP_PROP_FRAME_WIDTH, _information.GetInteger(KEY_WIDTH));

            if (_information.Contains(KEY_HEIGHT))
                _cap.set(CAP_PROP_FRAME_HEIGHT, _information.GetInteger(KEY_HEIGHT));

           Register_Pub("cam_image");

            // TODO: Add params like saturation, see http://wiki.ros.org/usb_cam 
        }



        void icvCvVideoCaptureSource::Execute()
    {
           // buff_Output(outputPorts);


            _cap >> mFrame;
		//tempdata->setoutvalue(mFrame);
            data::icvCvMatData datatosend(mFrame);
            icvPublish("cam_image",&datatosend);//=mFrame;
            //pub1->Send_Out(tempdata);

        }
    }
}
