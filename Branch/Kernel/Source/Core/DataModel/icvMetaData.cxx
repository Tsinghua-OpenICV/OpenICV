#include "OpenICV/Core/icvMetaData.h"
#include "OpenICV/Core/icvMacros.h"

#include <boost/variant/get.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace boost;

namespace icv { namespace core
{
    bool CompareContent(const icvMetaDataElementType& lhs, const icvMetaDataElementType& rhs)
    {
        // type compare
        if (lhs.which() != rhs.which())
            return false;

        // content compare
        switch (lhs.which())
        {
        case icvMetaDataElementString:
            if (get<string>(lhs) != get<string>(rhs)) return false;
            break;
        case icvMetaDataElementMap:
            if (*get<icvMetaData*>(lhs) != *get<icvMetaData*>(rhs)) return false;
            break;
        case icvMetaDataElementArray:
            if (*get<icvMetaDataArray*>(lhs) != *get<icvMetaDataArray*>(rhs)) return false;
            break;
        }

        return true;
    }

    icvMetaData& icvMetaData::operator=(const icvMetaData& data)
    {
        icv_set<string> keys = data.GetKeys();
        for (auto iter = keys.begin(); iter != keys.end(); iter++)
        {
            switch (data._values.at(*iter).which())
            {
            case icvMetaDataElementString:
                _values.emplace(*iter, data.GetString(*iter));
                break;
            case icvMetaDataElementMap:
                _values.emplace(*iter, new icvMetaData(data.GetMap(*iter)));
                break;
            case icvMetaDataElementArray:
                _values.emplace(*iter, new icvMetaDataArray(data.GetArray(*iter)));
                break;
            }
        }
        return *this;
    }
    void icvMetaData::FreePointers()
    {
        for (auto iter = _values.begin(); iter != _values.end(); iter++)
        {
            switch (iter->second.which())
            {
            case icvMetaDataElementString:
                break;
            case icvMetaDataElementMap:
                delete get<icvMetaData*>(iter->second);
                break;
            case icvMetaDataElementArray:
                delete get<icvMetaDataArray*>(iter->second);
                break;
            }
        }
    }
    bool icvMetaData::operator==(const icvMetaData& data)
    {
        // size compare
        if (Size() != data.Size())
            return false;
        for (auto iter = _values.begin(); iter != _values.end(); iter++)
        {
            // keys compare
            if (data._values.find(iter->first) == data._values.end())
                return false;

            // value compare
            if (!CompareContent(iter->second, data._values.at(iter->first)))
                return false;
        }
        return true;
    }

    bool icvMetaData::GetBoolean(const string& key) const
    {
        return lexical_cast<bool>(get<string>(_values.at(key)));
    }
    icvMetaData::SelfRef icvMetaData::SetBoolean(const string& key, const bool& value)
    {
        _values[key] = lexical_cast<string>(value);
        return *this;
    }
    bool icvMetaData::IsBoolean(const string& key) const
    {
        if (_values.at(key).type() != typeid(string))
            return false;
        try
        {
            boost::lexical_cast<bool>(get<string>(_values.at(key)));
            return true;
        }
        catch (...) { return false; }
    }
    int64_t icvMetaData::GetInteger(const string& key) const
    {
        return lexical_cast<int64_t>(get<string>(_values.at(key)));
    }
    icvMetaData::SelfRef icvMetaData::SetInteger(const string& key, const int64_t& value)
    {
        _values[key] = lexical_cast<string>(value);
        return *this;
    }
    bool icvMetaData::IsInteger(const string& key) const
    {
        if (_values.at(key).type() != typeid(string))
            return false;
        try
        {
            boost::lexical_cast<int64_t>(get<string>(_values.at(key)));
            return true;
        }
        catch (...) { return false; }
    }
    double_t icvMetaData::GetDecimal(const string& key) const
    {
        return lexical_cast<double_t>(get<string>(_values.at(key)));
    }
    icvMetaData::SelfRef icvMetaData::SetDecimal(const string& key, const double_t& value)
    {
        _values[key] = lexical_cast<string>(value);
        return *this;
    }
    bool icvMetaData::IsDecimal(const string& key) const
    {
        if (_values.at(key).type() != typeid(string))
            return false;
        try
        {
            boost::lexical_cast<double_t>(get<string>(_values.at(key)));
            return true;
        }
        catch (...) { return false; }
    }
    string& icvMetaData::GetString(const string& key)
    {
        return get<string>(_values.at(key));
    }
    icvMetaData::SelfRef icvMetaData::SetString(const string& key, const string& value)
    {
        _values[key] = value;
        return *this;
    }
    bool icvMetaData::IsString(const string& key) const
    {
        return _values.at(key).type() == typeid(string);
    }

    icvMetaDataArray& icvMetaData::GetArray(const string& key)
    {
        return *get<icvMetaDataArray*>(_values.at(key));
    }
    icvMetaData::SelfRef icvMetaData::AddArray(const string& key)
    {
        if (Contains(key))
        {
            if (_values.at(key).type() != typeid(icvMetaDataArray*))
                ICV_THROW_MESSAGE("Existing object with the key is not array");
        }
        else _values.emplace(key, new icvMetaDataArray);
        return *this;
    }
    bool icvMetaData::IsArray(const string& key) const
    {
        return _values.at(key).type() == typeid(icvMetaDataArray*);
    }
    icvMetaData& icvMetaData::GetMap(const string& key)
    {
        return *get<icvMetaData*>(_values.at(key));
    }
    icvMetaData::SelfRef icvMetaData::AddMap(const string& key)
    {
        if (Contains(key))
        {
            if (_values.at(key).type() != typeid(icvMetaData*))
                ICV_THROW_MESSAGE("Existing object with the key is not map");
        }
        else _values.emplace(key, new icvMetaData);
        return *this;
    }
    bool icvMetaData::IsMap(const string& key) const
    {
        return _values.at(key).type() == typeid(icvMetaData*);
    }

    void icvMetaData::Remove(const string& key)
    {
        if (Contains(key))
        {
            switch (_values.at(key).which())
            {
            case icvMetaDataElementString:
                break;
            case icvMetaDataElementMap:
                delete get<icvMetaData*>(_values.at(key));
                break;
            case icvMetaDataElementArray:
                delete get<icvMetaDataArray*>(_values.at(key));
                break;
            }
            _values.erase(key);
        }
    }
    void icvMetaData::Clear()
    {
        FreePointers();
        _values.clear();
    }
    icv_set<string> icvMetaData::GetKeys() const
    {
        icv_set<string> keys;
        keys.reserve(_values.size());
        for (auto iter = _values.begin(); iter != _values.end(); iter++)
            keys.emplace(iter->first);
        return keys;
    }

    icvMetaDataArray& icvMetaDataArray::operator=(const icvMetaDataArray& data)
    {
        for (int idx = 0; idx < data.Size(); idx++)
        {
            switch (data._values[idx].which())
            {
            case icvMetaDataElementString:
                _values.push_back(data.GetString(idx));
                break;
            case icvMetaDataElementMap:
                _values.push_back(new icvMetaData(data.GetMap(idx)));
                break;
            case icvMetaDataElementArray:
                _values.push_back(new icvMetaDataArray(data.GetArray(idx)));
                break;
            }
        }
        return *this;
    }
    void icvMetaDataArray::FreePointers()
    {
        for (auto iter = _values.begin(); iter != _values.end(); iter++)
        {
            switch (iter->which())
            {
            case icvMetaDataElementString:
                break;
            case icvMetaDataElementMap:
                delete get<icvMetaData*>(*iter);
                break;
            case icvMetaDataElementArray:
                delete get<icvMetaDataArray*>(*iter);
                break;
            }
        }
    }
    bool icvMetaDataArray::operator==(const icvMetaDataArray& data)
    {
        // size compare
        if (Size() != data.Size())
            return false;
        for (int idx = 0; idx < data.Size(); idx++)
        {
            // value compare
            if(!CompareContent(_values[idx], data._values[idx]))
                return false;
        }
        return true;
    }

    bool icvMetaDataArray::GetBoolean(const size_t& index) const
    {
        return lexical_cast<bool>(get<string>(_values[index]));
    }
    icvMetaDataArray::SelfRef icvMetaDataArray::SetBoolean(const size_t& index, const bool& value)
    {
        _values[index] = lexical_cast<string>(value);
        return *this;
    }
    bool icvMetaDataArray::IsBoolean(const size_t& index) const
    {
        if (_values[index].type() != typeid(string))
            return false;
        try
        {
            boost::lexical_cast<bool>(get<string>(_values[index]));
            return true;
        }
        catch (...) { return false; }
    }
    icvMetaDataArray::SelfRef icvMetaDataArray::AddBoolean(const bool& value)
    {
        _values.push_back(lexical_cast<string>(value));
        return *this;
    }
    int64_t icvMetaDataArray::GetInteger(const size_t& index) const
    {
        return lexical_cast<int64_t>(get<string>(_values[index]));
    }
    icvMetaDataArray::SelfRef icvMetaDataArray::SetInteger(const size_t& index, const int64_t& value)
    {
        _values[index] = lexical_cast<string>(value);
        return *this;
    }
    bool icvMetaDataArray::IsInteger(const size_t& index) const
    {
        if (_values[index].type() != typeid(string))
            return false;
        try
        {
            boost::lexical_cast<int64_t>(get<string>(_values[index]));
            return true;
        }
        catch (...) { return false; }
    }
    icvMetaDataArray::SelfRef icvMetaDataArray::AddInteger(const int64_t& value)
    {
        _values.push_back(lexical_cast<string>(value));
        return *this;
    }
    double_t icvMetaDataArray::GetDecimal(const size_t& index) const
    {
        return lexical_cast<double_t>(get<string>(_values[index]));
    }
    icvMetaDataArray::SelfRef icvMetaDataArray::SetDecimal(const size_t& index, const double_t& value)
    {
        _values[index] = lexical_cast<string>(value);
        return *this;
    }
    bool icvMetaDataArray::IsDecimal(const size_t& index) const
    {
        if (_values[index].type() != typeid(string))
            return false;
        try
        {
            boost::lexical_cast<double_t>(get<string>(_values[index]));
            return true;
        }
        catch (...) { return false; }
    }
    icvMetaDataArray::SelfRef icvMetaDataArray::AddDecimal(const double_t& value)
    {
        _values.push_back(lexical_cast<string>(value));
        return *this;
    }
    string& icvMetaDataArray::GetString(const size_t& index)
    {
        return get<string>(_values[index]);
    }
    icvMetaDataArray::SelfRef icvMetaDataArray::SetString(const size_t& index, const string& value)
    {
        _values[index] = value;
        return *this;
    }
    bool icvMetaDataArray::IsString(const size_t& index) const
    {
        return _values[index].type() == typeid(string);
    }
    icvMetaDataArray::SelfRef icvMetaDataArray::AddString(const string& value)
    {
        _values.push_back(value);
        return *this;
    }

    icvMetaDataArray& icvMetaDataArray::GetArray(const size_t& index)
    {
        return *get<icvMetaDataArray*>(_values[index]);
    }
    icvMetaDataArray::SelfRef icvMetaDataArray::AddArray()
    {
        _values.push_back(new icvMetaDataArray);
        return *this;
    }
    bool icvMetaDataArray::IsArray(const size_t& index) const
    {
        return _values[index].type() == typeid(icvMetaDataArray*);
    }
    icvMetaData& icvMetaDataArray::GetMap(const size_t& index)
    {
        return *get<icvMetaData*>(_values[index]);
    }
    icvMetaDataArray::SelfRef icvMetaDataArray::AddMap()
    {
        _values.push_back(new icvMetaData);
        return *this;
    }
    bool icvMetaDataArray::IsMap(const size_t& index) const
    {
        return _values[index].type() == typeid(icvMetaData*);
    }

    void icvMetaDataArray::Remove(const size_t& index)
    {
        if (index < _values.size())
        {
            if (_values[index].type() == typeid(icvMetaDataArray*))
                delete get<icvMetaDataArray*>(_values[index]);
            else if (_values[index].type() == typeid(icvMetaData*))
                delete get<icvMetaData*>(_values[index]);
            _values.erase(_values.begin() + index);
        }
    }
    void icvMetaDataArray::Clear()
    {
        FreePointers();
        _values.clear();
    }
}}
