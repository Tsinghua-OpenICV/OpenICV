icv_set_module_vars(FeatureTracker)
#INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} )
find_package(OpenCV REQUIRED ) 
include_directories(${OpenICV_ini_path}/Branch/Kernel/Include)
#add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})
add_library(${MODULE_TARGET_NAME} SHARED feature_tracker.cpp feature_tracker.h)

find_package(Torch REQUIRED)
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES( ${MODULE_TARGET_NAME} 
icvCore 

${TORCH_LIBRARIES}
${OpenCV_LIBS} 
${Boost_LIBRARIES})

icv_set_solution_folder(icvFeatureTracker Perception)
icv_install_module(FeatureTracker)
