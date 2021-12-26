
#Prerequsites
#find_package(Boost REQUIRED log filesystem system)
add_definitions(-DBOOST_LOG_DYN_LINK)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
#add_subdirectory(PlaneGroundFilter)
#add_subdirectory(FeatureTracker)
add_subdirectory(LaneDetectionMT)
add_subdirectory(LaneDepartureWarning)
add_subdirectory(ForwardCollisionWarning)
add_subdirectory(LaneChangeWarning)
add_subdirectory(SensorFusion)
#add_subdirectory(Darknet_ICV)





