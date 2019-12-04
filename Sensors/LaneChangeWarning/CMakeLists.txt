icv_set_module_vars(LaneChangeWarning)

find_package(OpenCV REQUIRED ) 
add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${EIGEN_INCLUDE_DIR})


target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES( ${MODULE_TARGET_NAME} 
icvCore 
icvExtensionOpenCV
${OpenCV_LIBS} 
${Boost_LIBRARIES})

icv_set_solution_folder(icvLaneChangeWarning Sensors)
icv_install_module(LaneChangeWarning)
