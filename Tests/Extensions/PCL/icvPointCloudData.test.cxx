#include <boost/test/unit_test.hpp>

#include "OpenICV/Extensions/PCL/icvPointCloudData.hxx"

#include "pcl/point_types.h"

namespace boost
{
    namespace test_tools
    {
        namespace tt_detail
        {
            bool operator != (const ::pcl::PointXYZ& lhs, const ::pcl::PointXYZ& rhs)
            {
                return lhs.x != rhs.x || lhs.y != rhs.y || lhs.z != rhs.z;
            }
        }
    }
}

BOOST_AUTO_TEST_SUITE(icvPointCloudData)

typedef icv::pcl::icvPointCloudData<::pcl::PointXYZ> DXYZ;
typedef ::pcl::PointCloud<::pcl::PointXYZ> PCXYZ;

::pcl::PointXYZ p1(0, 0, 0);
::pcl::PointXYZ p2(0, 1, 1);
::pcl::PointXYZ p3(-1, 0, -1);

BOOST_AUTO_TEST_CASE(constructor)
{
    DXYZ data(3);
    data.Reserve();
}

BOOST_AUTO_TEST_CASE(assign)
{
    DXYZ data(3);
    data.Reserve();

    PCXYZ pc(3, 1);
    pc.at(0) = p1;
    pc.at(1) = p2;
    pc.at(2) = p3;
    data = pc;

    const PCXYZ& dref = data;
    BOOST_CHECK_EQUAL_COLLECTIONS(dref.begin(), dref.end(), pc.begin(), pc.end());
}

BOOST_AUTO_TEST_CASE(copy)
{
    DXYZ data(3);
    data.Reserve();

    PCXYZ pc(3, 1);
    pc.at(0) = p1;
    pc.at(1) = p2;
    pc.at(2) = p3;
    data = pc;

    uint32_t expect_memsize = data.GetActualMemorySize();

    DXYZ* copy = static_cast<DXYZ*>(data.DeepCopy());
    const PCXYZ& dref = data;
    const PCXYZ& cref = *copy;
    BOOST_CHECK_EQUAL_COLLECTIONS(dref.begin(), dref.end(), pc.begin(), pc.end());
    BOOST_CHECK_EQUAL(copy->GetActualMemorySize(), expect_memsize);

    copy->Dispose();
    BOOST_CHECK_EQUAL(copy->GetActualMemorySize(), 0);
    BOOST_CHECK_EQUAL(data.GetActualMemorySize(), expect_memsize);

    delete copy;
}

BOOST_AUTO_TEST_SUITE_END()
