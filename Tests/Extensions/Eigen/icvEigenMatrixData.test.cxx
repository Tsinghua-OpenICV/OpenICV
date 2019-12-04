#include <boost/test/unit_test.hpp>

#include "OpenICV/Extensions/Eigen/icvEigenMatrixData.hxx"

namespace boost
{
    namespace test_tools
    {
        namespace tt_detail
        {
        }
    }
}

BOOST_AUTO_TEST_SUITE(icvEigenMatrixData)

typedef icv::eigen::icvEigenMatrixData<double, 2, 2> FixedType;
typedef icv::eigen::icvEigenVectorXdData VectorType;
typedef icv::eigen::icvEigenMatrixXdData MatrixType;

BOOST_AUTO_TEST_CASE(constructor)
{
    FixedType fixed;
    VectorType vector(3);
    MatrixType matrix(3, 3);

    fixed.Reserve();
    vector.Reserve();
    matrix.Reserve();
}

BOOST_AUTO_TEST_CASE(assign)
{
    FixedType fixed;
    VectorType vector(3);
    MatrixType matrix(3, 3);

    fixed.Reserve();
    vector.Reserve();
    matrix.Reserve();

    FixedType::UType dfix;
    dfix << 0, 1, 2, 3;
    VectorType::UType dvec(3);
    dvec << 0, 1, 2;
    MatrixType::UType dmat(3, 3);
    dmat << 0, 1, 2, 3, 4, 5, 6, 7, 8;

    fixed = dfix;
    vector = dvec;
    matrix = dmat;

    const FixedType::UType& rfix = fixed;
    BOOST_CHECK_EQUAL(rfix, dfix);
    const VectorType::UType& rvec = vector;
    BOOST_CHECK_EQUAL(rvec, dvec);
    const MatrixType::UType& rmat = matrix;
    BOOST_CHECK_EQUAL(rmat, dmat);
}

BOOST_AUTO_TEST_CASE(copy)
{
    FixedType data;
    data.Reserve();
    FixedType::UType dfix;
    dfix << 0, 1, 2, 3;
    data = dfix;

    uint32_t expect_memsize = data.GetActualMemorySize();

    FixedType* copy = static_cast<FixedType*>(data.DeepCopy());
    const FixedType::UType& dref = data;
    const FixedType::UType& cref = *copy;
    BOOST_CHECK_EQUAL(dref, cref);
    BOOST_CHECK_EQUAL(copy->GetActualMemorySize(), expect_memsize);

    copy->Dispose();
    BOOST_CHECK_EQUAL(copy->GetActualMemorySize(), 0);
    BOOST_CHECK_EQUAL(data.GetActualMemorySize(), expect_memsize);

    delete copy;
}

BOOST_AUTO_TEST_SUITE_END()
