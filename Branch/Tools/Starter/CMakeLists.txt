find_package(Boost COMPONENTS  program_options  system filesystem log REQUIRED)

add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions(-DBOOST_LOG_DYN_LINK)
add_executable(icvStarter Entry.cxx)
target_link_libraries(icvStarter PUBLIC icvCore icvEngine icvBasis pthread ${Boost_LIBRARIES} msgpackc)
include_directories(${OpenICV_SOURCE_DIR}/Include
)
target_include_directories(icvStarter SYSTEM PUBLIC 
$<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>  
    $<INSTALL_INTERFACE:>)
icv_install_module(Starter)
