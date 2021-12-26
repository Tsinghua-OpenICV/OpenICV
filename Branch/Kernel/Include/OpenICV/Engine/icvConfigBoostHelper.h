#ifndef icvConfigBoostHelper_h
#define icvConfigBoostHelper_h

#include <boost/property_tree/ptree.hpp>

namespace icv { namespace core
{
    class icvMetaData;
    class icvMetaDataArray;

    icvMetaData& operator<<(icvMetaData& data, const boost::property_tree::ptree& tree);
    icvMetaDataArray& operator<<(icvMetaDataArray& data, const boost::property_tree::ptree& tree);
}}

#endif // icvConfigBoostHelper_h
