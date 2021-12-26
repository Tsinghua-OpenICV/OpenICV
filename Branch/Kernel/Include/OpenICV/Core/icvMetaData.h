#ifndef icvMetaData_h
#define icvMetaData_h

#define ICV_CONFIG_STDINT
#define ICV_CONFIG_CONTAINERS
#define ICV_CONFIG_POINTERS
#include "OpenICV/Core/icvConfig.h"
#undef ICV_CONFIG_STDINT
#undef ICV_CONFIG_CONTAINERS
#undef ICV_CONFIG_POINTERS

#include <vector>
#include <boost/variant/variant.hpp>

#include "OpenICV/Core/icvObject.h"

namespace icv { namespace core
{
    class icvMetaData;
    class icvMetaDataArray;

    // Not using recursive_variant here for manual pointer management
    typedef boost::variant<std::string, icvMetaData*, icvMetaDataArray*> icvMetaDataElementType;
    enum { // Expansion of icvMetaDataElementType::which()
        icvMetaDataElementString = 0,
        icvMetaDataElementMap = 1,
        icvMetaDataElementArray = 2,
    };

    // MetaData acts as associate container for any basic types
    // TODO: Add initializer_list support and interator
    // TODO: Add move initializer?
    // TODO: Add built-in support for integer and Decimal
    class icvMetaData : public icvObject
    {
    public:
        typedef icv_shared_ptr<icvMetaData> Ptr;
        typedef icvMetaData& SelfRef;

        ICV_DEFAULTED_CONSTRUCTOR(icvMetaData);
        icvMetaData(const icvMetaData& data) { operator=(data); }
        ~icvMetaData() { FreePointers(); }
        icvMetaData& operator = (const icvMetaData& data);
        bool operator == (const icvMetaData& data);
        bool operator != (const icvMetaData& data) { return !operator==(data); }

        bool GetBoolean(const std::string& key) const;
        SelfRef SetBoolean(const std::string& key, const bool& value);
        bool IsBoolean(const std::string& key) const;

        int64_t GetInteger(const std::string& key) const;
        SelfRef SetInteger(const std::string& key, const int64_t& value);
        bool IsInteger(const std::string& key) const;

        double_t GetDecimal(const std::string& key) const;
        SelfRef SetDecimal(const std::string& key, const double_t& value);
        bool IsDecimal(const std::string& key) const;

        std::string& GetString(const std::string& key);
        const std::string& GetString(const std::string& key) const
        { return const_cast<icvMetaData&>(*this).GetString(key); }
        SelfRef SetString(const std::string& key, const std::string& value);
        bool IsString(const std::string& key) const;

        icvMetaDataArray& GetArray(const std::string& key);
        const icvMetaDataArray& GetArray(const std::string& key) const
        { return const_cast<icvMetaData&>(*this).GetArray(key); }
        SelfRef AddArray(const std::string& key);
        bool IsArray(const std::string& key) const;

        icvMetaData& GetMap(const std::string& key);
        const icvMetaData& GetMap(const std::string& key) const
        { return const_cast<icvMetaData&>(*this).GetMap(key); }
        SelfRef AddMap(const std::string& key);
        bool IsMap(const std::string& key) const;

        // Not encouraged to use these accessors
        icvMetaDataElementType& Get(const std::string& key) { return _values.at(key); }
        const icvMetaDataElementType& Get(const std::string& key) const { return _values.at(key); }
        SelfRef Set(const std::string& key, icvMetaDataElementType& value) { _values[key] = value; return *this; }

        icv_set<std::string> GetKeys() const;
        void Remove(const std::string& key);
        bool Contains(const std::string& key) const { return _values.count(key); }
        void Clear();
        size_t Size() const { return _values.size(); }

    private:
        void FreePointers();

    private:
        icv_map<std::string, icvMetaDataElementType> _values;
    };

    // Help to store array values in MetaData
    // TODO: Add initializer_list support and interator
    class icvMetaDataArray : public icvObject
    {
    public:
        typedef icv_shared_ptr<icvMetaDataArray> Ptr;
        typedef icvMetaDataArray& SelfRef;

        ICV_DEFAULTED_CONSTRUCTOR(icvMetaDataArray);
        icvMetaDataArray(const icvMetaDataArray& data) { operator=(data); }
        ~icvMetaDataArray() { FreePointers(); }
        icvMetaDataArray& operator = (const icvMetaDataArray& data);
        bool operator == (const icvMetaDataArray& data);
        bool operator != (const icvMetaDataArray& data) { return !operator==(data); }

        bool GetBoolean(const size_t& index) const;
        SelfRef SetBoolean(const size_t& index, const bool& value);
        bool IsBoolean(const size_t& index) const;
        SelfRef AddBoolean(const bool& value);

        int64_t GetInteger(const size_t& index) const;
        SelfRef SetInteger(const size_t& index, const int64_t& value);
        bool IsInteger(const size_t& index) const;
        SelfRef AddInteger(const int64_t& value);

        double_t GetDecimal(const size_t& index) const;
        SelfRef SetDecimal(const size_t& index, const double_t& value);
        bool IsDecimal(const size_t& index) const;
        SelfRef AddDecimal(const double_t& value);

        std::string& GetString(const size_t& index);
        const std::string& GetString(const size_t& index) const
        { return const_cast<icvMetaDataArray&>(*this).GetString(index); }
        SelfRef SetString(const size_t& index, const std::string& value);
        bool IsString(const size_t& index) const;
        SelfRef AddString(const std::string& value);

        icvMetaDataArray& GetArray(const size_t& index);
        const icvMetaDataArray& GetArray(const size_t& index) const
        { return const_cast<icvMetaDataArray&>(*this).GetArray(index); }
        bool IsArray(const size_t& index) const;
        SelfRef AddArray();

        icvMetaData& GetMap(const size_t& index);
        const icvMetaData& GetMap(const size_t& index) const
        { return const_cast<icvMetaDataArray&>(*this).GetMap(index); }
        bool IsMap(const size_t& index) const;
        SelfRef AddMap();

        // Not encouraged to use these accessors
        icvMetaDataElementType& Get(const size_t& index) { return _values.at(index); }
        const icvMetaDataElementType& Get(const size_t& index) const { return _values.at(index); }
        SelfRef Set(const size_t& index, icvMetaDataElementType& value) { _values[index] = value; return *this; }

        void Remove(const size_t& index);
        void Clear();
        size_t Size() const { return _values.size(); }

    private:
        void FreePointers();

    private:
        std::vector<icvMetaDataElementType> _values;
    };
}}

#endif // icvMetaData_h
