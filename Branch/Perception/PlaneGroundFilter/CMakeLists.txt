icv_set_module_vars(PlaneGroundFilter)
add_compile_options(-std=c++11)
include_directories(${OpenICV_ini_path}/Branch/Kernel/Include)
find_package(PCL 1.8 REQUIRED)
add_library(${MODULE_TARGET_NAME} SHARED PlaneGroundFilter.cpp PlaneGroundFilter.h)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES( ${MODULE_TARGET_NAME} 
icvCore 
${PCL_LIBRARIES}
${Boost_LIBRARIES})

icv_set_solution_folder(icvPlaneGroundFilter Perception)
icv_install_module(PlaneGroundFilter)