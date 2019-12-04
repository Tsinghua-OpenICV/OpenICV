icv_set_module_vars(PylonCam)
add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)
add_definitions(-DBOOST_LOG_DYN_LINK)


add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})

#find_package(Pylon QUIET)
#if (NOT ${Pylon_FOUND})
#    include("${CMAKE_CURRENT_SOURCE_DIR}/FindPylon.cmake")
#endif()   

set(Pylon_INCLUDE_DIRS /opt/pylon5/include)
include_directories(${Pylon_INCLUDE_DIRS})
set(Pylon_LIB_DIR /opt/pylon5/lib64)
link_directories(${Pylon_LIB_DIR})




target_link_libraries(${MODULE_TARGET_NAME} 
    PUBLIC icvCore icvExtensionOpenCV 
    ${Boost_LIBRARIES}
    #${Pylon_LIBRARIES}	
    pylonbase
    pylonutility
    GenApi_gcc_v3_0_Basler_pylon_v5_0
    GCBase_gcc_v3_0_Basler_pylon_v5_0
    ${OpenCV_LIBS} 
  )
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})

icv_set_solution_folder(icvPylonCam Sensors)

icv_install_module(PylonCam)

