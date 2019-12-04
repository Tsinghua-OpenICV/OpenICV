#include "OpenICV/Engine/icvConfigBoostHelper.h"
#include "OpenICV/Core/icvMetaData.h"
#include <boost/property_tree/ptree.hpp>

namespace pt = boost::property_tree;

namespace icv { namespace core
{
    icvMetaData& operator<<(icvMetaData& data, const boost::property_tree::ptree& tree)
    {
        for (auto item : tree)
        {
            std::string key = item.first;
            if (item.second.data().length() == 0)
            {
                auto subtree = item.second.get_child("");
                if (subtree.begin()->first.length() == 0)
                {
                    // Assume the subtree is an array when the node name of first subnode is empty
                    data.AddArray(key);
                    data.GetArray(key) << subtree;
                }
                else
                {
                    data.AddMap(key);
                    data.GetMap(key) << subtree;
                }
            }
            else data.SetString(key, item.second.data());
        }
        return data;
    }

    icvMetaDataArray& operator<<(icvMetaDataArray& data, const boost::property_tree::ptree& tree)
    {
        for (auto item : tree)
        {
            if (item.second.data().length() == 0)
            {
                auto subtree = item.second.get_child("");
                if (subtree.begin()->first.length() == 0)
                {
                    // Assume the subtree is an array when the node name of first subnode is empty
                    data.AddArray();
                    data.GetArray(data.Size() - 1) << subtree;
                }
                else
                {
                    data.AddMap();
                    data.GetMap(data.Size() - 1) << subtree;
                }
            }
            else data.AddString(item.second.data());
        }
        return data;
    }
}}
