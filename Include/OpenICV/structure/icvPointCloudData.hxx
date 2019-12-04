#ifndef icvPointCloudData_hxx
#define icvPointCloudData_hxx

#include "OpenICV/Core/icvDataObject.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace icv
{
    namespace pcl
    {
        template <typename PointT>
        class icvPointCloudData : public icv::core::icvDataObject
        {
        public:
            typedef ::pcl::PointCloud<PointT> UType;
            typedef icvPointCloudData<PointT> SelfType;

        public:
            // Point value argument could be used after initialization
            icvPointCloudData(uint32_t count) : _init_width(count), _init_height(1) {}
            icvPointCloudData(uint32_t width, uint32_t height) : _init_width(width), _init_height(height) {}
            icvPointCloudData() : _init_width(100), _init_height(1) {Reserve();}

            virtual void Reserve() ICV_OVERRIDE
            {
                if(!_data) {_data = new UType(_init_width, _init_height);is_reserved=true;}
            }
            virtual void Dispose() ICV_OVERRIDE
            {
                if (_data) delete _data;
                _data = ICV_NULLPTR;
            }

            virtual Uint64 GetActualMemorySize() ICV_OVERRIDE
            {
                if (_data) return _data->size() * sizeof(PointT);
                else return 0;
            }

            virtual void Serialize(std::ostream& out, const uint32_t& version) const ICV_OVERRIDE
            {
                ICV_THROW_MESSAGE("Directly serialize icvPointCloudData is not supported, please convert it to icvTableData");
            }
            virtual void Deserialize(std::istream& in, const uint32_t& version) ICV_OVERRIDE
            {
                ICV_THROW_MESSAGE("Directly serialize icvPointCloudData is not supported, please convert it from icvTableData");
            }

            virtual icv::core::icvDataObject* DeepCopy() ICV_OVERRIDE
            {
                SelfType* copy = new SelfType(_init_width, _init_height);

                copy->_sourceTime = _sourceTime;
                copy->_init_width = _init_width;
                copy->_init_height = _init_height;
                copy->Reserve(); *(copy->_data) = *_data;
                return copy;
            }
         virtual void CopyTo(icvDataObject* dst)  ICV_OVERRIDE {

                SelfType* copy = static_cast< SelfType*>(dst);
                copy->_sourceTime = _sourceTime;
                copy->_init_width = _init_width;
                copy->_init_height = _init_height;
                copy->Reserve(); *(copy->_data) = *_data;



         };
         



            SelfType * get_Ptr()
        {

            return static_cast< SelfType*> (this);

        }
        void setoutvalue(const UType& data)
            {
                 _assign(data);
                this->SetSourceTime(SyncClock::now_us().time_since_epoch().count());


            }
            virtual std::string Print() ICV_OVERRIDE
            {
                return "pcl::PointCloud size:(" +
                    std::to_string(_init_width) + " x " + std::to_string(_init_height) + ")";
            }

            operator UType&() { return *_data; }
            operator const UType&() const { return *_data; }
           
            UType* operator->() { return _data; }
            const UType* operator->() const { return _data; }

            icvPointCloudData<PointT>& operator = (const UType& data)
            {
                _assign(data);
                return *this;
            }

        private:
            UType * _data = ICV_NULLPTR;

            // Params only for initialization
            uint32_t _init_width, _init_height;

        private:
            void _assign(const ::pcl::PointCloud<PointT>& data)
            {
                // Copy properties
                _init_width = data.width;
                _init_height = data.height;

                // Copy data
                *_data = data;
            }
            
        };
    }
}

#endif // icvPointCloudData_hxx
