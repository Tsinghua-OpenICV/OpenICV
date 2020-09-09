icv_set_module_vars(SickLDMRS)
find_package(Eigen3 REQUIRED)


SET (LDMRS_HEADERS
  BasicDatatypes.hpp
	application/LdmrsApp.hpp
	application/LdmrsFieldApp.hpp
	application/LdmrsSectorChangeApp.hpp
	application/LdmrsNtpTimeApp.hpp
	application/LdmrsScanpointCoordinateApp.hpp
	application/BasicApplication.hpp
	devices/BasicDevice.hpp
	devices/LD_MRS.hpp
	devices/LuxBase.hpp
	sopas/LdmrsSopasLayer.hpp
	sopas/SopasBase.hpp
	sopas/colaa.hpp
	sopas/colab.hpp
	datatypes/Measurement.hpp
	datatypes/SensorStateInfo.hpp
	datatypes/Fields.hpp
	datatypes/FieldParameter.hpp
	datatypes/FieldDescription.hpp
	datatypes/EvalCase.hpp
	datatypes/EvalCases.hpp
	datatypes/EvalCaseResult.hpp
	datatypes/EvalCaseResults.hpp
	datatypes/Trigger.hpp
	datatypes/Point2D.hpp
	datatypes/Point3D.hpp
	datatypes/Position3D.hpp
	datatypes/PositionWGS84.hpp
	datatypes/Polygon2D.hpp
	datatypes/Polygon3D.hpp
	datatypes/Circle2D.hpp
	datatypes/Ellipse2D.hpp
	datatypes/Box2D.hpp
	datatypes/Line2D.hpp
	datatypes/Msg.hpp
	datatypes/Scan.hpp
	datatypes/ScanPoint.hpp
	datatypes/ScannerInfo.hpp
	datatypes/Object.hpp
	interfaces/tcp.hpp
	interfaces/file.hpp
	tools/errorhandler.hpp
	tools/SickThread.hpp
	tools/toolbox.hpp
	tools/MathToolbox.hpp
	tools/Mutex.hpp
	tools/Timer.hpp
	tools/Time.hpp
	tools/WatchdogTimer.hpp
	tools/BasicDataBuffer.hpp
	manager.hpp
)

SET (LDMRS_SOURCES
	application/LdmrsApp.cpp
	application/LdmrsFieldApp.cpp
	application/LdmrsSectorChangeApp.cpp
	application/LdmrsNtpTimeApp.cpp
	application/LdmrsScanpointCoordinateApp.cpp
	application/BasicApplication.cpp
	devices/BasicDevice.cpp
	devices/LD_MRS.cpp
	devices/LuxBase.cpp
	sopas/LdmrsSopasLayer.cpp
	sopas/SopasBase.cpp
	sopas/colaa.cpp
	sopas/colab.cpp
	datatypes/Measurement.cpp
	datatypes/SensorStateInfo.cpp
	datatypes/Fields.cpp
	datatypes/FieldParameter.cpp
	datatypes/FieldDescription.cpp
	datatypes/EvalCase.cpp
	datatypes/EvalCases.cpp
	datatypes/EvalCaseResult.cpp
	datatypes/EvalCaseResults.cpp
	datatypes/Polygon2D.cpp
	datatypes/Polygon3D.cpp
	datatypes/Trigger.cpp
	datatypes/Point2D.cpp
	datatypes/Point3D.cpp
	datatypes/Position3D.cpp
	datatypes/PositionWGS84.cpp
	datatypes/Circle2D.cpp
	datatypes/Ellipse2D.cpp
	datatypes/Box2D.cpp
	datatypes/Line2D.cpp
	datatypes/Msg.cpp
	datatypes/Scan.cpp
	datatypes/ScanPoint.cpp
	datatypes/ScannerInfo.cpp
	datatypes/Object.cpp
	interfaces/tcp.cpp
	interfaces/file.cpp
	tools/errorhandler.cpp
	tools/SickThread.cpp
	tools/Mutex.cpp
	tools/toolbox.cpp
	tools/MathToolbox.cpp
	tools/BasicDataBuffer.cpp
	tools/Timer.cpp
	tools/Time.cpp
	tools/WatchdogTimer.cpp
	manager.cpp
	icvSick.cxx
)

# Add the binary dir so that the UI files are wrapped correctly
include_directories (${CMAKE_CURRENT_BINARY_DIR})
include_directories (${CMAKE_CURRENT_SOURCE_DIR})

# Set CMake policy
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)



# The target library
add_library(icvSickLDMRS SHARED
  ${LDMRS_SOURCES}
  ${LDMRS_HEADERS})
target_link_libraries(icvSickLDMRS icvCore pthread ${Boost_LIBRARIES})

#set_target_properties(icvSickLDMRS PROPERTIES
# VERSION ${SICK_LDMRS_VERSION} SOVERSION ${SICK_LDMRS_MAJOR_VERSION})


icv_set_solution_folder(icvSickLDMRS Sensors)

icv_install_module(SickLDMRS)

