icv_set_module_vars(PyFrontend)
#find_package(Boost REQUIRED log)
add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)
include_directories(${OpenICV_ini_path}/Branch/Kernel/Include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_definitions(-DBOOST_LOG_DYN_LINK)
set(Python2_INCLUDE_DIRS /usr/include/python2.7)
set(Python2_LIBRARY /usr/lib/x86_64-linux-gnu/libpython2.7.so)
add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})

target_link_libraries(${MODULE_TARGET_NAME} PUBLIC icvCore  ${Boost_LIBRARIES} ${Python2_LIBRARY} msgpackc)
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS}  ${Python2_INCLUDE_DIRS} )

#message(" booost===   ${Boost_LIBRARIES}")
#message(" opencv===   ${OpenCV_LIBS}")
icv_set_solution_folder(icvPyFrontend Sensors)

icv_install_module(PyFrontend)
