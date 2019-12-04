#include "OpenICV/Extensions/OpenCV/icvCvVideoCaptureSource.h"
#include "OpenICV/structure/icvCvMatData.hxx"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeOutput.h"
#include "OpenICV/Core/icvNodeInput.h"
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
            : icvFunction(0, 1, info)
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

            // TODO: Add params like saturation, see http://wiki.ros.org/usb_cam 
        }

        // void icvCvVideoCaptureSource::ConfigurateOutput(std::vector<icvNodeOutput*>& outputPorts)
        // {
        //      ini_Output(outputPorts,1);

        //     if (!outputPorts[0]->RequireDataObject())
        //     {
        //         ICV_LOG_TRACE << "Instantiated" << boost::typeindex::type_id<icvCvMatData>().name() << " at output";

        //         Mat tem=Mat(Size(_cap.get(CAP_PROP_FRAME_WIDTH),_cap.get(CAP_PROP_FRAME_HEIGHT)),CV_8UC3);
        //         icvDataObject* outimage=new icvCvMatData(_cap.get(CAP_PROP_FRAME_HEIGHT), _cap.get(CAP_PROP_FRAME_WIDTH), CV_8UC3 );
        //         outimage->Reserve();
        //         outputPorts[0]->SetDataObject(outimage);

        //     }
        // }

        void icvCvVideoCaptureSource::Execute(std::vector<icvNodeInput*>& inputPorts, std::vector<icvNodeOutput*>& outputPorts)
    {
           // buff_Output(outputPorts);


            _cap >> mFrame;
		tempdata->setoutvalue(mFrame);

            Send_Out(tempdata,0);

        }
    }
}
