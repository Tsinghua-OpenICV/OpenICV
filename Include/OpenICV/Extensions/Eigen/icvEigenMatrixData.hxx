#ifndef icvEigenMatrixData_hxx
#define icvEigenMatrixData_hxx

#include "OpenICV/Core/icvDataObject.h"

#include <Eigen/Core>
#include <boost/utility/enable_if.hpp>

namespace icv
{
    namespace eigen
    {
        template <typename _Scalar, int _Rows, int _Cols,
            int _Options = Eigen::AutoAlign | 
                ( (_Rows == 1 && _Cols != 1) ? Eigen::RowMajor
                : (_Cols == 1 && _Rows != 1) ? Eigen::ColMajor
                : EIGEN_DEFAULT_MATRIX_STORAGE_ORDER_OPTION),
            int _MaxRows = _Rows, int _MaxCols = _Cols>
        class icvEigenMatrixData : icv::core::icvDataObject
        {
        public:
            typedef Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> UType;
            typedef icvEigenMatrixData<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> SelfType;

        public:
            icvEigenMatrixData()
            {
                BOOST_STATIC_ASSERT_MSG(UType::SizeAtCompileTime != Eigen::Dynamic,
                    "You shouldn't initialize a dynamic matrix without a shape.");
            }
            icvEigenMatrixData(uint32_t dim) : _init_arg1(dim)
            {
                BOOST_STATIC_ASSERT_MSG((UType::IsVectorAtCompileTime && UType::SizeAtCompileTime == Eigen::Dynamic),
                    "You should initialize a dynamic vector giving a dimension.");
            }
            icvEigenMatrixData(uint32_t rows, uint32_t cols) : _init_arg1(rows), _init_arg2(cols)
            {
                // FIXME: Following statement doesn;t work properly...
                // BOOST_STATIC_ASSERT_MSG((!UType::IsVectorAtCompileTime && UType::SizeAtCompileTime == Eigen::Dynamic),
                //     "You should initialize a dynamic matrix giving a shape.");
            }

            virtual void Reserve() ICV_OVERRIDE
            {
                if (!_data)
                {
                    _new();
                }
            }
            virtual void Dispose() ICV_OVERRIDE
            {
                if (_data) delete _data;
                _data = ICV_NULLPTR;
            }

            virtual Uint64 GetActualMemorySize() ICV_OVERRIDE
            {
                if (_data) return _data->rows() * _data->cols() * sizeof(_Scalar);
                else return 0;
            }

            virtual void Serialize(std::ostream& out, const uint32_t& version) const ICV_OVERRIDE
            {
                ICV_THROW_MESSAGE("Directly serialize icvEigenMatrixData is not supported, please convert it to icvTensorData");
            }
            virtual void Deserialize(std::istream& in, const uint32_t& version) ICV_OVERRIDE
            {
                ICV_THROW_MESSAGE("Directly serialize icvEigenMatrixData is not supported, please convert it from icvTensorData");
            }

            virtual icv::core::icvDataObject* DeepCopy() ICV_OVERRIDE
            {
                SelfType* copy = new SelfType(_init_arg1, _init_arg2);
                copy->_sourceTime = _sourceTime;
                copy->_init_arg1 = _init_arg1;
                copy->_init_arg2 = _init_arg2;
                copy->Reserve(); *(copy->_data) = *_data;
                return copy;
            }

            virtual std::string Print() ICV_OVERRIDE
            {
                return "Eigen::Matrix";
            }

            operator const UType&() const { return *_data; }

            UType* operator->() { return _data; }
            const UType* operator->() const { return _data; }

            SelfType& operator = (const UType& data)
            {
                *_data = data;
                return *this;
            }

        private:
            UType * _data = ICV_NULLPTR;

            // Params only for initialization
            uint32_t _init_arg1, _init_arg2;

        private:
            template<class M = UType, typename boost::enable_if<boost::integral_constant<bool,
                M::IsVectorAtCompileTime && M::SizeAtCompileTime == Eigen::Dynamic>, int>::type = 0>
            void _new()
            {
                _data = new M(_init_arg1);
            }

            template<class M = UType, typename boost::enable_if<boost::integral_constant<bool,
                !M::IsVectorAtCompileTime && M::SizeAtCompileTime == Eigen::Dynamic>, int>::type = 0>
            void _new()
            {
                _data = new M(_init_arg1, _init_arg2);
            }

            template<class M = UType, typename boost::enable_if<boost::integral_constant<bool,
                M::SizeAtCompileTime != Eigen::Dynamic>, int>::type = 0>
            void _new()
            {
                _data = new M();
            }
        };

        #define EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)                             \
        /** \ingroup matrixtypedefs */                                                              \
        typedef icvEigenMatrixData<Type, Size, Size> icvEigenMatrix##SizeSuffix##TypeSuffix##Data;  \
        /** \ingroup matrixtypedefs */                                                              \
        typedef icvEigenMatrixData<Type, Size, 1>    icvEigenVector##SizeSuffix##TypeSuffix##Data;  \
        /** \ingroup matrixtypedefs */                                                              \
        typedef icvEigenMatrixData<Type, 1, Size>    icvEigenRowVector##SizeSuffix##TypeSuffix##Data;

        #define EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, Size)                                         \
        /** \ingroup matrixtypedefs */                                                                    \
        typedef icvEigenMatrixData<Type, Size, Eigen::Dynamic> icvEigenMatrix##Size##X##TypeSuffix##Data; \
        /** \ingroup matrixtypedefs */                                                                    \
        typedef icvEigenMatrixData<Type, Eigen::Dynamic, Size> icvEigenMatrix##X##Size##TypeSuffix##Dara;

        #define EIGEN_MAKE_TYPEDEFS_ALL_SIZES(Type, TypeSuffix) \
                EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 2, 2) \
                EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 3, 3) \
                EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 4, 4) \
                EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Eigen::Dynamic, X) \
                EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 2) \
                EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 3) \
                EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 4)

        EIGEN_MAKE_TYPEDEFS_ALL_SIZES(int, i)
        EIGEN_MAKE_TYPEDEFS_ALL_SIZES(float, f)
        EIGEN_MAKE_TYPEDEFS_ALL_SIZES(double, d)
        // EIGEN_MAKE_TYPEDEFS_ALL_SIZES(std::complex<float>, cf)
        // EIGEN_MAKE_TYPEDEFS_ALL_SIZES(std::complex<double>, cd)

        #undef EIGEN_MAKE_TYPEDEFS_ALL_SIZES
        #undef EIGEN_MAKE_TYPEDEFS
        #undef EIGEN_MAKE_FIXED_TYPEDEFS
    }
}

#endif // icvEigenMatrixData_hxx
