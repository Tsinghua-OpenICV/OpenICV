icv_set_module_vars(SensorFusion)

SET (SOURCE_
    icvFusion.cxx
    objects/obsf_define.h

    objects/obsf_header.h
    
    objects/obsf_obs_vector.h
    objects/obsf_obs2d.h
    objects/obsf_obs3d.h
    objects/obsf_obs_frame.h
    objects/obsf_fusion_track.h
    objects/obsf_fusion_track.cpp
    objects/obsf_track.h
    objects/obsf_track.cpp

    filters/base_filter_wrap.h
    #filters/cov_table.h
    #filters/kalman_filter_wrap.h
    #filters/kalman_filter_wrap.cpp

    filters/fusion_kalman_filter.h
    filters/fusion_kalman_filter.cpp

    filters/obsf_uncertainty_fusion_engine.h
    filters/obsf_uncertainty_fusion_engine.cpp 


    #visualization/obsf_pcl_visual.cpp
    #visualization/obsf_pcl_visual.h
    #visualization/pcl_visual_util.cpp
    #visualization/pcl_visual_util.h
    #visualization/ros_util.h
)



# Add the binary dir so that the UI files are wrapped correctly
include_directories (${CMAKE_CURRENT_BINARY_DIR})
include_directories (${CMAKE_CURRENT_SOURCE_DIR})
#find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

INCLUDE_DIRECTORIES ( ${EIGEN_INCLUDE_DIRS} )

# Set CMake policy
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)

add_definitions(-DBOOST_LOG_DYN_LINK)
# find_package(OpenCV REQUIRED ) 



# The target library
add_library(icvSensorFusion SHARED
  ${SOURCE_})
target_link_libraries(icvSensorFusion icvCore icvExtensionOpenCV ${Eigen_LIBRARIES}
    ${OpenCV_LIBS} ${Boost_LIBRARIES} pthread)

target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})
#set_target_properties(icvSickLDMRS PROPERTIES
# VERSION ${SICK_LDMRS_VERSION} SOVERSION ${SICK_LDMRS_MAJOR_VERSION})


icv_set_solution_folder(icvSensorFusion Sensors)

icv_install_module(SensorFusion)



















