#include "OpenICV/Extensions/OpenCV/icvCvNamedWindowSink.h"
#include "OpenICV/structure/icvCvMatData.hxx"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeOutput.h"
#include "OpenICV/Core/icvNodeInput.h"
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
            : icvFunction(1, 0, info)
        {
            if (_information.Contains(KEY_WINDOW_NAME))
                _name = _information.GetString(KEY_WINDOW_NAME);
            else
                _name = "anonymous" + to_string(_name_counter++);
        }

        // void icvCvNamedWindowSink::ConfigurateInput(std::vector<icvNodeInput*>& inputPorts)
        // {
        //     if (!inputPorts[0]) ICV_THROW_MESSAGE("At least one input is needed!");
        //     // TODO: how to check input type
        // }

        void icvCvNamedWindowSink::Execute(std::vector<icvNodeInput*>& inputPorts, std::vector<icvNodeOutput*>& outputPorts)
    {
        icvDataObject** inData = new icvDataObject*[inputPorts.size()];
        for (int i = 0; i < inputPorts.size(); i++)
        {
            inData[i] = inputPorts[i]->RequireDataObject();
        }
        icvDataObject** outData = new icvDataObject*[outputPorts.size()];
        for (int i = 0; i < outputPorts.size(); i++)
        {
            outData[i] = outputPorts[i]->WriteDataObject();
        }
            if (!_created)
            {
                // Window should be created in the same thread as imshow
                namedWindow(_name);
                _created = true;
            }

            cv::Mat& image = inData[0]->As<icvCvMatData>();
            imshow(_name, image);

            waitKey(1);
        }
    }
}