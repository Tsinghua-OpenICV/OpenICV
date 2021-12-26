icv_set_module_vars(LidarRobosens)

file(GLOB_RECURSE ${MODULE_HDR_NAME} *.h)
find_package(PCL REQUIRED ) 
find_package(rs_driver REQUIRED)
include_directories(${OpenICV_ini_path}/Branch/Kernel/Include)
add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})
#list(APPEND CMAKE_MODULE_PATH ./rs_driver/cmake)
#include_directories(${rs_driver_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

target_link_libraries(${MODULE_TARGET_NAME} icvCore ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${rs_driver_LIBRARIES})
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${rs_driver_INCLUDE_DIRS})

icv_set_solution_folder(${MODULE_TARGET_NAME} Sensors)

icv_install_module(LidarRobosens)



        