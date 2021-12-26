
#Prerequsites
#find_package(Boost REQUIRED log filesystem system)
add_definitions(-DBOOST_LOG_DYN_LINK)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)

#add_subdirectory(G3Dynamics)
#add_subdirectory(TuanDynamics)
add_subdirectory(XiaopengDynamics)




