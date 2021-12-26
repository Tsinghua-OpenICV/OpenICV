#ifndef icvPointCloudData_hxx
#define icvPointCloudData_hxx

#include "OpenICV/Core/icvDataObject.h"
#include <msgpack.hpp>
#include <msgpack/versioning.hpp>
#include <msgpack/adaptor/adaptor_base.hpp>
#include <msgpack/meta.hpp>
#include <msgpack/object.hpp>
//#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct PointXYZI {
    float x;
    float y;
    float z;
    float intensity;
    MSGPACK_DEFINE(x,y,z,intensity)
};
struct PointCloudForTransmission {
    float width;
    float height;
    bool is_dense;
    std::vector<PointXYZI> points;
    MSGPACK_DEFINE(width,height,is_dense,points)
};

namespace icv
{
    namespace data
    {
        // struct PointCloudRecStruct
        // {			
        //     string imagename;
        //     string another_name;
        //     MSGPACK_DEFINE(imagename,another_name);	
        // } ;
        template <typename PointT>
        class icvPointCloudData : public icv::core::icvDataObject
        {
        public:
            typedef ::pcl::PointCloud<PointT> UType;
            typedef icvPointCloudData<PointT> SelfType;
           struct PointCloudRecStruct
        {			
            string pointcloudname;
            string another_name;
            MSGPACK_DEFINE(pointcloudname,another_name);	
        } ;
        public:
            // Point value argument could be used after initialization
            icvPointCloudData(uint32_t count) : _init_width(count), _init_height(1) {}
            icvPointCloudData(uint32_t width, uint32_t height) : _init_width(width), _init_height(height) {}
            icvPointCloudData() : _init_width(100), _init_height(1) {Reserve();}
            icvPointCloudData(UType data_ini) : _init_width(100), _init_height(1) {Reserve();setoutvalue(data_ini);}

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
                if (_data) return sizeof(PointT);
               // if (_data) return _data->size() * sizeof(PointT);
                else return 0;
            }

    //         virtual void Serialize(std::stringstream& out, const uint32_t& version) const ICV_OVERRIDE
    //         {      
    //             PointCloudForTransmission cloud_trans;
    //             cloud_trans.width=_data->width;
    //             cloud_trans.is_dense=_data->is_dense;
    //             cloud_trans.height=_data->height;
    //             cloud_trans.points.resize(_data->width*_data->height);


    //             for (int i=0;i<_data->points.size();i++){
    //             //std::cout<<"point x: "<<cld->points[i].x<<" y: "<<cld->points[i].y<<" z: "<<cld->points[i].z<<endl;
    //             cloud_trans.points[i].x=_data->points[i].x;
    //             cloud_trans.points[i].y=_data->points[i].y;
    //             cloud_trans.points[i].z=_data->points[i].z;
    //             //cloud_trans.points[i].intensity=_data->points[i].intensity;
                
    //             }
    //                 msgpack::pack(out, cloud_trans);

    //         }
    //         virtual void Deserialize(std::stringstream& in, const uint32_t& version) ICV_OVERRIDE
    //         {
           
    //     std::string const& str = in.str();
    //     msgpack::object_handle oh =
    //         msgpack::unpack(str.data(), str.size());
    //     msgpack::object obj = oh.get();
    //     //std::cout <<"the object is "<< obj << std::endl;
    //   PointCloudForTransmission cloud_get=obj.as<PointCloudForTransmission>();
        
    //     _data->width=cloud_get.width;
    //     _data->is_dense=cloud_get.is_dense;
    //     _data->height=cloud_get.height;
    //     _data->points.resize(cloud_get.width*cloud_get.height);


    // for (int i=0;i<cloud_get.points.size();i++){
    //   //std::cout<<"point x: "<<cld->points[i].x<<" y: "<<cld->points[i].y<<" z: "<<cld->points[i].z<<endl;
    //   _data->points[i].x=cloud_get.points[i].x;
    //   _data->points[i].y=cloud_get.points[i].y;
    //   _data->points[i].z=cloud_get.points[i].z;
    //   //_data->points[i].intensity=cloud_get.points[i].intensity;
    //   }

    //         }
            virtual void Serialize(std::stringstream& out, const uint32_t& version) const ICV_OVERRIDE
            {
                 std::stringstream buff ;
                msgpack::pack(buff, _data_rec);


               // out <<this->GetSourceTime()<<" "<<buff.size()<<" "<< buff.data();
                out <<this->GetSourceTime()<<" "<<buff.str().size()<<" " << buff.str().data();

             //   ICV_LOG_INFO<<"data:"<<_data_rec.imagename <<"RECORD LENGTH"<<buff.size();

            }
            virtual void Deserialize(std::stringstream& in, const uint32_t& version) ICV_OVERRIDE
            {

            
            size_t len;
            string buff;
            time_t timestamp;
            in >> timestamp>>len >> buff;   
            this->SetSourceTime(timestamp);
            if(len==0) _data=ICV_NULLPTR;
                else 
                {    
            msgpack::object_handle oh =msgpack::unpack(buff.data(), buff.size());      
                msgpack::object obj = oh.get();
                PointCloudRecStruct temp_V;
                obj.convert(temp_V);
                _data_rec.pointcloudname=temp_V.pointcloudname;
                }
            }
            virtual icv::core::icvDataObject* DeepCopy() ICV_OVERRIDE
            {
                SelfType* copy = new SelfType(_init_width, _init_height);

                copy->_sourceTime = _sourceTime;
                copy->_init_width = _init_width;
                copy->_init_height = _init_height;
                copy->Reserve(); *(copy->_data) = *_data;
                copy->_data_rec.pointcloudname=_data_rec.pointcloudname;
                return copy;
            }
         virtual void CopyTo(icvDataObject* dst)  ICV_OVERRIDE {

                SelfType* copy = static_cast< SelfType*>(dst);
                copy->_sourceTime = _sourceTime;
                copy->_init_width = _init_width;
                copy->_init_height = _init_height;
                copy->Reserve(); *(copy->_data) = *_data;
                copy->_data_rec.pointcloudname=_data_rec.pointcloudname;



         };
         

           string getPointCloudName()
        {
            return _data_rec.pointcloudname;
        }

            SelfType * get_Ptr()
        {

            return static_cast< SelfType*> (this);

        }
        void setvalue(const UType& data)
            {
                 _assign(data);
            time_t temp_time=icvTime::time_us(); 
                this->SetSourceTime(temp_time);
                  
            _data_rec.pointcloudname=to_string(temp_time);
               

            }
            virtual std::string Print() ICV_OVERRIDE
            {
                return "pcl::PointCloud size:(" +
                    std::to_string(_init_width) + " x " + std::to_string(_init_height) + ")";
            }

            operator UType&() { return *_data; }
            operator const UType&() const { return *_data; }
           
            UType* operator->() { return _data; }
            UType getvalue() { return *_data; }
            const UType* operator->() const { return _data; }

            icvPointCloudData<PointT>& operator = (const UType& data)
            {
                _assign(data);
                return *this;
            }

        private:
            UType * _data = ICV_NULLPTR;
            PointCloudRecStruct _data_rec ;
            // Params only for initialization
            uint32_t _init_width, _init_height;

        
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
