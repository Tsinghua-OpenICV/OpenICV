#ifndef icvMacro_h
#define icvMacro_h

/**********   Interface   **********/

#include <boost/dll/alias.hpp>
#define ICV_FUNCTION_EXPORT_PREFIX icvFunc_
// Should be used outside any namespaces
#define ICV_FUNCTION_EXPORT(functionClass) BOOST_DLL_ALIAS(functionClass, ICV_FUNCTION_EXPORT_PREFIX ## functionClass)

/**********  Basic Types  **********/
#define ICV_CONFIG_STDINT
#include "OpenICV/Core/icvConfig.h" // import <cstdint
#undef ICV_CONFIG_STDINT

#include "OpenICV/Core/icvTime.h" // import time data types

namespace icv
{
    // XXX: add support for void (or NaN)?
    // XXX: add support for complex? (e.g. Complex64 = std::complex<Float32>)
    typedef bool Boolean;
    typedef std::int8_t Int8;
    typedef std::int16_t Int16;
    typedef std::int32_t Int32;
    typedef std::int64_t Int64;
    typedef std::uint8_t Uint8;
    typedef std::uint16_t Uint16;
    typedef std::uint32_t Uint32;
    typedef std::uint64_t Uint64;
    typedef std::float_t Float32;
    typedef std::double_t Float64;
    typedef std::size_t IndexType;
    typedef std::ptrdiff_t IndexDiffType;
}

#define ICV_BASIC_TYPES_TEMPLATE(statement)\
    statement(Boolean)      \
    statement(Int8)         \
    statement(Int16)        \
    statement(Int32)        \
    statement(Int64)        \
    statement(Uint8)        \
    statement(Uint16)       \
    statement(Uint32)       \
    statement(Uint64)       \
    statement(Float32)      \
    statement(Float64)      \
    statement(Duration32)   \
    statement(Duration64)   \

// #include <boost/type_traits/is_arithmetic.hpp>
#define ICV_CHECK_BASIC_TYPE(type) BOOST_STATIC_ASSERT((boost::is_arithmetic<type>::value   \
    || ::boost::is_same<type, ::icv::Duration32>::value                                     \
    || ::boost::is_same<type, ::icv::Duration64>::value))

/********** Getter/Setter **********/

#define ICV_PROPERTY_GET(name, variable, type)          \
    type Get##name() { return variable; }               \
    const type Get##name() const { return variable; }   \

#define ICV_PROPERTY_GETSET(name, variable, type)       \
    ICV_PROPERTY_GET(name, variable, type)              \
    void Set##name(const type& name) { variable = name; }

#define ICV_PROPERTY_GETSET_PTR(name, variable, ptr)    \
    ICV_PROPERTY_GET(name, variable, ptr)               \
    void Set##name(ptr name) { variable = name; }

/**********    Loggers    **********/

#include <boost/log/trivial.hpp>

#define ICV_LOG_TRACE BOOST_LOG_TRIVIAL(trace)
#define ICV_LOG_DEBUG BOOST_LOG_TRIVIAL(debug)
#define ICV_LOG_INFO  BOOST_LOG_TRIVIAL(info)
#define ICV_LOG_WARN  BOOST_LOG_TRIVIAL(warning)
#define ICV_LOG_ERROR BOOST_LOG_TRIVIAL(error)
#define ICV_LOG_FATAL BOOST_LOG_TRIVIAL(fatal)

/**********   Exception   **********/

#include <boost/throw_exception.hpp>

#define ICV_THROW_MESSAGE(msg) BOOST_THROW_EXCEPTION(std::runtime_error(msg))

#endif // icvMacro_h