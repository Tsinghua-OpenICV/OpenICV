#include <boost/test/unit_test.hpp>

#include "OpenICV/Extensions/OpenCV/icvCvMatData.h"

BOOST_AUTO_TEST_SUITE(icvCvMatData)

using namespace icv;
using namespace icv::data;
typedef icv::data::icvCvMatData DType;

BOOST_AUTO_TEST_CASE(constructor)
{
    cv::Mat mat(3, 3, CV_8UC1);
    DType data(3, 3, CV_8UC1);
    data.Reserve();
    data = mat;
}

BOOST_AUTO_TEST_CASE(assign)
{
    Uint8 raw[9] = { 0, 1, 2, 3, 4, 5, 6, 7, 8 };
    cv::Mat mat(3, 3, CV_8UC1, raw);

    DType data(3, 3, CV_8UC1);
    data.Reserve();
    data = mat;

    const cv::Mat& dref = data;
    BOOST_CHECK_EQUAL_COLLECTIONS(dref.begin<Uint8>(), dref.end<Uint8>(),
        mat.begin<Uint8>(), mat.end<Uint8>());
}

BOOST_AUTO_TEST_CASE(copy)
{
    Uint8 raw[9] = { 0, 1, 2, 3, 4, 5, 6, 7, 8 };
    cv::Mat mat(3, 3, CV_8UC1, raw);
    DType data(3, 3, CV_8UC1);
    data.Reserve();
    data = mat;

    uint32_t expect_memsize = data.GetActualMemorySize();

    DType* copy = static_cast<DType*>(data.DeepCopy());
    const cv::Mat& dref = data;
    const cv::Mat& cref = *copy;
    BOOST_CHECK_EQUAL_COLLECTIONS(dref.begin<Uint8>(), dref.end<Uint8>(),
        cref.begin<Uint8>(), cref.end<Uint8>());
    BOOST_CHECK_EQUAL(copy->GetActualMemorySize(), expect_memsize);

    copy->Dispose();
    BOOST_CHECK_EQUAL(copy->GetActualMemorySize(), 0);
    BOOST_CHECK_EQUAL(data.GetActualMemorySize(), expect_memsize);

    delete copy;
}

BOOST_AUTO_TEST_SUITE_END()
