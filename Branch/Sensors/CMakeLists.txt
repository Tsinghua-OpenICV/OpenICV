
#Prerequsites
#find_package(Boost REQUIRED log filesystem system)
add_definitions(-DBOOST_LOG_DYN_LINK)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)





#add_subdirectory(TuanDynamics)
#add_subdirectory(Mobileye)
#add_subdirectory(SickLDMRS)

#add_subdirectory(Navigation)
#add_subdirectory(LidarPandar)
#add_subdirectory(LidarRobosens)

#add_subdirectory(XiaopengDynamics)
add_subdirectory(RemoteSensors)

#add_subdirectory(DSRC_transfer)
#add_subdirectory(zmqTest) #Deprecated
#add_subdirectory(helaRadar)
add_subdirectory(LaneDetectionMT)
#add_subdirectory(LaneDetectionCV)
add_subdirectory(RecordReplay_old)
#Succeeded:
add_subdirectory(Gps)
add_subdirectory(LidarHDL)
add_subdirectory(SimpleCam)
add_subdirectory(ReadVideo)
add_subdirectory(IMUEthernet)
add_subdirectory(RadarConti)
add_subdirectory(RecordReplay)
add_subdirectory(LaneDepartureWarning)
add_subdirectory(GpsEthernet)
add_subdirectory(VideoComm)
add_subdirectory(ForwardCollisionWarning)
add_subdirectory(LaneChangeWarning)
add_subdirectory(SensorFusion)
add_subdirectory(Trajectory_tracking)
add_subdirectory(Display)
add_subdirectory(Mobileye)

#Failed:
#add_subdirectory(SSD)   #Lack header file dnndk.h!

#add_subdirectory(PylonCam)  #No pylon header file!
add_subdirectory(Navigation)  #No ADNCoordConverter.cpp functions are not defined!
#add_subdirectory(DelayTest)
#Deprecated
#add_subdirectory(RemoteComm)





