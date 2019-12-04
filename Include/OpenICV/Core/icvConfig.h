#include <boost/config.hpp>

#ifndef icvConfig_h_common
#define icvConfig_h_common

// #ifdef _WIN32
// #include <SDKDDKVer.h> // supply windows version information to boost
// #endif

/********** Macros to support old compilers **********/
#define ICV_CONSTEXPR BOOST_CONSTEXPR_OR_CONST
#define ICV_CONSTEXPR_FUNC BOOST_CONSTEXPR

#define ICV_NOEXCEPT BOOST_NOEXCEPT
#define ICV_NOEXCEPT_OR_NOTHROW BOOST_NOEXCEPT_OR_NOTHROW

#if defined(BOOST_NO_CXX11_FINAL)
# define ICV_FINAL
# define ICV_OVERRIDE
#else
# define ICV_FINAL final
# define ICV_OVERRIDE override
#endif

#define ICV_DEFAULTED_CONSTRUCTOR(cls) BOOST_DEFAULTED_FUNCTION(cls(), {})

#if defined(BOOST_NO_CXX11_NULLPTR)
const class nullptr_t
{
public:
    template<class T>           operator T*() const {return 0;}
    template<class C, class T>  operator T C::*() const { return 0; }
private:
    void operator& () const;
} nullptr_i = {};
# define ICV_NULLPTR nullptr_i
#else
# define ICV_NULLPTR nullptr
#endif

#if defined(BOOST_NO_CXX11_SCOPED_ENUMS)
# define ICV_ENUM_CLASS struct
#else
# define ICV_ENUM_CLASS enum class
#endif

#endif // icvConfig_h_common

/********** Implementation selection **********/
// TOOD: use template alias to define the classes.
//       (However, this brought in additional compatibility issues)

// stdint.h
#ifdef ICV_CONFIG_STDINT

#ifndef icvConfig_h_stdint
#define icvConfig_h_stdint

#if defined(BOOST_HAS_STDINT_H) || defined(OPENICV_PREFER_BOOST)
# include <boost/cstdint.hpp>
#else
# include <cstdint>
#endif
#include <boost/cstdfloat.hpp>

#endif // icvConfig_h_stdint
#endif // ICV_CONFIG_STDINT

// associate containers
#ifdef ICV_CONFIG_CONTAINERS

#ifndef icvConfig_h_containers
#define icvConfig_h_containers

#if defined(OPENICV_PREFER_BOOST)
# include <boost/unordered_map.hpp>
# include <boost/unordered_set.hpp>
# define icv_map ::boost::unordered_map
# define icv_set ::boost::unordered_set
#elif defined(BOOST_NO_CXX11_STD_UNORDERED)
# include <map>
# include <set>
# define icv_map ::std::map
# define icv_set ::std::set
#else
# include <unordered_map>
# include <unordered_set>
# define icv_map ::std::unordered_map
# define icv_set ::std::unordered_set
#endif

#endif // icvConfig_h_containers
#endif // ICV_CONFIG_CONTAINERS

#ifdef ICV_CONFIG_TUPLE

#ifndef icvConfig_h_tuple
#define icvconfig_h_tuple

#if defined(BOOST_NO_CXX11_HDR_TUPLE) || defined(OPENICV_PREFER_BOOST)
# include <boost/tuple/tuple.hpp>
# define icv_tuple ::boost::tuple
# define icv_make_tuple ::boost::make_tuple
#else
# include <tuple>
# define icv_tuple ::std::tuple
# define icv_make_tuple ::std::make_tuple
#endif

#endif // icvConfig_h_tuple
#endif // ICV_CONFIG_TUPLE

// smart pointers
#ifdef ICV_CONFIG_POINTERS

#ifndef icvConfig_h_pointers
#define icvConfig_h_pointers

#if defined(BOOST_NO_CXX11_SMART_PTR) || defined(OPENICV_PREFER_BOOST)
# include <boost/shared_ptr.hpp>
# include <boost/make_shared.hpp>
# define icv_shared_ptr ::boost::shared_ptr
# define icv_make_shared ::boost::make_shared
#else
# include <memory>
# define icv_shared_ptr ::std::shared_ptr
# define icv_make_shared ::std::make_shared
#endif

#endif // icvConfig_h_pointers
#endif // ICV_CONFIG_POINTERS

// function
#ifdef ICV_CONFIG_FUNCTION

#ifndef icvConfig_h_function
#define icvConfig_h_function

#if defined(BOOST_NO_CXX11_HDR_FUNCTIONAL) || defined(OPENICV_PREFER_BOOST)
# include <boost/function.hpp>
# define icv_function ::boost::function
# define icv_bind ::boost::bind
#else
# include <functional>
# define icv_function ::std::function
# define icv_bind ::std::bind
#endif

#endif // icvConfig_h_function
#endif // ICV_CONFIG_FUNCTION

// time and calendar
#ifdef ICV_CONFIG_TIME

#ifndef icvConfig_h_time
#define icvConfig_h_time

#if defined(BOOST_NO_CXX11_HDR_CHRONO) || defined(OPENICV_PREFER_BOOST)
# include <boost/chrono.hpp>
# define icv_ratio ::boost::ratio
# define icv_chrono ::boost::chrono
#else
# include <chrono>
# define icv_ratio ::std::ratio
# define icv_chrono ::std::chrono
#endif

#endif // icvConfig_h_time
#endif // ICV_CONFIG_TIME

// threading
#ifdef ICV_CONFIG_THREAD

#ifndef icvConfig_h_thread
#define icvConfig_h_thread

#if defined(BOOST_NO_CXX11_HDR_THREAD) || defined(OPENICV_PREFER_BOOST)
# include <boost/thread/thread.hpp>
# define icv_thread ::boost::thread
# define icv_this_thread ::boost::this_thread

# include <boost/thread/mutex.hpp>
# define icv_mutex ::boost::mutex
# define icv_unique_lock ::boost::unique_lock

# include <boost/thread/condition_variable.hpp>
# define icv_condition_variable ::boost::condition_variable
#else
# include <thread>
# define icv_thread ::std::thread
# define icv_this_thread ::std::this_thread

# include <mutex>
# define icv_mutex ::std::mutex
# define icv_unique_lock ::std::unique_lock

# include <condition_variable>
# define icv_condition_variable ::std::condition_variable
#endif

#endif // icvConfig_h_thread
#endif // ICV_CONFIG_THREAD

// typeid and type_index
#ifdef ICV_CONFIG_TYPEINDEX

#ifndef icvConfig_h_typeinfo
#define icvConfig_h_typeinfo

#if defined(BOOST_NO_TYPEID) || defined(OPENICV_PREFER_BOOST)
# include <boost/type_index.hpp>
# define icv_typeid(type) ::boost::typeindex::type_id<type>()
# define icv_type_index ::boost::typeindex::type_index
#else
# include <typeindex>
# define icv_typeid(type) typeid(type)
# define icv_type_index ::std::type_index
#endif

#endif // icvConfig_h_typeinfo
#endif // ICV_CONFIG_TYPEINDEX

// variant 
#ifdef ICV_CONFIG_VARIANT

#ifndef icvConfig_h_variant
#define icvConfig_h_variant

// XXX: BOOST_NO_CXX17_HDR_VARIANT is not available currently
#if defined(BOOST_NO_CXX17_HDR_VARIANT) || defined(OPENICV_PREFER_BOOST)
# include <boost/variant/variant.hpp>
# define icv_variant ::boost::variant
# define icv_variant_idx(var) var.which()
#else
# include <variant>
# define icv_variant ::std::variant
# define icv_variant_idx(var) var.index()
#endif

#endif // icvConfig_h_variant
#endif // ICV_CONFIG_VARIANT

#ifdef ICV_CONFIG_ATOMIC

#ifndef icvConfig_h_atomic
#define icvConfig_h_atomic

#if defined(BOOST_NO_CXX11_HDR_ATOMIC) || defined(OPENICV_PREFER_BOOST)
# include <boost/atomic.hpp>
# define icv_atomic ::boost::atomic
#else
# include <atomic>
# define icv_atomic ::std::atomic
#endif

namespace icv
{
    class spin_lock
    {
    private:
        typedef enum { Locked, Unlocked } LockState;
        icv_atomic<LockState> state_;

    public:
        spin_lock() : state_(Unlocked) {}

        void lock()
        {
            while (state_.exchange(Locked, boost::memory_order_acquire) == Locked);
        }
        void unlock()
        {
            state_.store(Unlocked, boost::memory_order_release);
        }
    };
}

#endif // icvConfig_h_atomic
#endif // ICV_CONFIG_ATOMIC
