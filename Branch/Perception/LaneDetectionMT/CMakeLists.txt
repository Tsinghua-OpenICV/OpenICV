icv_set_module_vars(LaneDetectionCV)
#INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} )
find_package(OpenCV REQUIRED ) 
include_directories(${OpenICV_ini_path}/Branch/Kernel/Include)
add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})
#message("source files : ${${MODULE_SRC_NAME}}")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/armadillo/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/armadillo/lib)

target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES( ${MODULE_TARGET_NAME} 
icvCore 

${CMAKE_CURRENT_SOURCE_DIR}/armadillo/lib/blas_win32_MT.lib	
${CMAKE_CURRENT_SOURCE_DIR}/armadillo/lib/lapack_win32_MT.lib 
${OpenCV_LIBS} 

${Boost_LIBRARIES})

icv_set_solution_folder(icvLaneDetectionCV Perception)
icv_install_module(LaneDetectionCV)
