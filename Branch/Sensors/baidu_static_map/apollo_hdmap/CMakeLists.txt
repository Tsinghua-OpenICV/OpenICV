cmake_minimum_required(VERSION 3.0.0)
project(hdmap)
set(PROJECT_VERSION_MAJOR 0)
set(PROJECT_VERSION_MINOR 1)
set(PROJECT_VERSION_PATCH 0)
set(PROJECT_VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})

# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC ${CMAKE_CXX_FLAGS}")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG")
if(NOT MSVC)
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")
endif()



# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Protobuf REQUIRED)


file(GLOB protobuf_files
	modules/map/proto/*.proto
	modules/map/relative_map/proto/*.proto
	modules/perception/proto/*.proto
	modules/common/proto/*.proto
	modules/localization/proto/*.proto
	modules/common/vehicle_state/proto/*.proto
	modules/routing/proto/*.proto
)

SET(PROTO_META_BASE_DIR ${CMAKE_CURRENT_BINARY_DIR})
LIST(APPEND PROTO_FLAGS -I${CMAKE_CURRENT_SOURCE_DIR})

set(PROTO_SRCS  )
set(PROTO_HDRS  )

message("### protoc = ${PROTOBUF_PROTOC_EXECUTABLE}")
message("### Protobuf_INCLUDE_DIRS = ${Protobuf_INCLUDE_DIRS}, Protobuf_LIBRARIES = ${Protobuf_LIBRARIES}")
FOREACH(FIL ${protobuf_files})
	GET_FILENAME_COMPONENT(FIL_WE ${FIL} NAME_WE)
	string(REGEX REPLACE ".+/(.+)\\..*" "\\1" FILE_NAME ${FIL})
	string(REGEX REPLACE "(.+)\\${FILE_NAME}.*" "\\1" FILE_PATH ${FIL})
	string(REGEX MATCH "(modules/map/proto/|modules/map/relative_map/proto/|modules/perception/proto/|modules/common/proto/|modules/localization/proto/)" OUT_PATH ${FILE_PATH})
	EXECUTE_PROCESS(
		COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ${PROTO_FLAGS} --cpp_out=${PROTO_META_BASE_DIR} ${FIL}
	)
	if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${OUT_PATH}${FIL_WE}.pb.cc)
		LIST(APPEND PROTO_SRCS ${CMAKE_CURRENT_BINARY_DIR}/${OUT_PATH}${FIL_WE}.pb.cc)
		LIST(APPEND PROTO_HDRS ${CMAKE_CURRENT_BINARY_DIR}/${OUT_PATH}${FIL_WE}.pb.h)
	endif ()
ENDFOREACH()


set(COMMON_SRCS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/modules/common)
set(MATH_SRCS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/modules/common/math)
set(MAP_SRCS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/modules/map)
set(ADAPTER_SRCS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/modules/map/hdmap/adapter)
set(hdmap_SRCS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/modules/map/hdmap)
set(PNC_MAP_SRCS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/modules/map/pnc_map)
set(CYBER_SRCS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/cyber)
set(LOCALIZATION_SRCS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/modules/localization)
set(PERCEPTION_SRCS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/modules/perception)

# include abseil path
 include_directories(
 	${CMAKE_SOURCE_DIR}/./install/include
 )
 link_directories(${CMAKE_SOURCE_DIR}/./install/lib)



#find_package(absl REQUIRED)
include_directories(
	${CMAKE_CURRENT_BINARY_DIR}
	${Boost_INCLUDE_DIRS}
	${absl_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}
	${Protobuf_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
)

add_subdirectory(abseil-cpp)

# if (EXISTS /usr/local/cuda/bin/nvcc OR EXISTS /usr/cuda/bin/nvcc)
# 	add_definitions(-D_USE_CUDA_)
# 	find_package(CUDA)
# 	include(FindCUDA)
# 	set(CUDA_ARCH_LIST Auto CACHE STRING
# 			"List of CUDA architectures (e.g. Pascal, Volta, etc) or \
# 	compute capability versions (6.1, 7.0, etc) to generate code for. \
# 	Set to Auto for automatic detection (default)."
# 	)
# 	cuda_select_nvcc_arch_flags(CUDA_ARCH_FLAGS ${CUDA_ARCH_LIST})
# 	list(APPEND CUDA_NVCC_FLAGS ${CUDA_ARCH_FLAGS})
# 	include_directories(
# 		${CUDA_INCLUDE_DIRS}	
# 	)
# 	cuda_add_library (pnc_cuda_util STATIC
# 		${PNC_MAP_SRCS_ROOT}/cuda_util.cu
# 	)
# 	target_link_libraries(pnc_cuda_util
# 		${CUDA_LIBRARIES}
# 		glog
# 		gflags
# 	)	
# else ()
# 	add_library (pnc_cuda_util STATIC
# 		${PNC_MAP_SRCS_ROOT}/cuda_util.cc
# 	)
# 	target_link_libraries(pnc_cuda_util
# 		glog
# 		gflags
# 	)	
# endif ()

add_library (hdmap SHARED
	## proto file
	${PROTO_SRCS}
	${PROTO_HDRS}

	## math
	# geometry
	${MATH_SRCS_ROOT}/vec2d.cc	
	${MATH_SRCS_ROOT}/math_utils.cc	
	${MATH_SRCS_ROOT}/math_utils.h	
	${MATH_SRCS_ROOT}/line_segment2d.cc	
	${MATH_SRCS_ROOT}/aabox2d.cc
	${MATH_SRCS_ROOT}/box2d.cc
	${MATH_SRCS_ROOT}/polygon2d.cc
	# sin_table
	${MATH_SRCS_ROOT}/sin_table.cc
	# angle
	${MATH_SRCS_ROOT}/angle.cc
	# euler_angles_zxy
	# ${MATH_SRCS_ROOT}/euler_angles_zxy.h
	# quaternion
	# ${MATH_SRCS_ROOT}/quaternion.h
	# matrix_operations
	${MATH_SRCS_ROOT}/matrix_operations.cc
	# kalman_filter
	# ${MATH_SRCS_ROOT}/kalman_filter.h
	# extended_kalman_filter
	# ${MATH_SRCS_ROOT}/extended_kalman_filter.h
	# factorial
	# ${MATH_SRCS_ROOT}/factorial.h
	# path_matcher
	${MATH_SRCS_ROOT}/path_matcher.cc
	# search
	${MATH_SRCS_ROOT}/search.cc
	# integral
	${MATH_SRCS_ROOT}/integral.cc	
	# linear_interpolation
	${MATH_SRCS_ROOT}/linear_interpolation.cc
	# nonlinear_interpolation
	# ${MATH_SRCS_ROOT}/nonlinear_interpolation.cc
	# linear_quadratic_regulator
	${MATH_SRCS_ROOT}/linear_quadratic_regulator.cc
	# cartesian_frenet_conversion
	${MATH_SRCS_ROOT}/cartesian_frenet_conversion.cc
	# hermite_spline
	# ${MATH_SRCS_ROOT}/hermite_spline.h

	## common/util
	${COMMON_SRCS_ROOT}/util/string_util.cc
	${COMMON_SRCS_ROOT}/util/util.cc
	# ${COMMON_SRCS_ROOT}/util/file.cc
	${CYBER_SRCS_ROOT}/common/file.cc

	## hdmap adapter
	${ADAPTER_SRCS_ROOT}/coordinate_convert_tool.cc
	${ADAPTER_SRCS_ROOT}/opendrive_adapter.cc
	${ADAPTER_SRCS_ROOT}/proto_organizer.cc
	${ADAPTER_SRCS_ROOT}/xml_parser/header_xml_parser.cc
	${ADAPTER_SRCS_ROOT}/xml_parser/junctions_xml_parser.cc
	${ADAPTER_SRCS_ROOT}/xml_parser/lanes_xml_parser.cc
	${ADAPTER_SRCS_ROOT}/xml_parser/objects_xml_parser.cc
	${ADAPTER_SRCS_ROOT}/xml_parser/roads_xml_parser.cc
	${ADAPTER_SRCS_ROOT}/xml_parser/signals_xml_parser.cc
	${ADAPTER_SRCS_ROOT}/xml_parser/util_xml_parser.cc

	## hdmap
	${hdmap_SRCS_ROOT}/hdmap.cc
	${hdmap_SRCS_ROOT}/hdmap_common.cc
	${hdmap_SRCS_ROOT}/hdmap_impl.cc

	# ${PNC_MAP_SRCS_ROOT}/path.cc
	# ${PNC_MAP_SRCS_ROOT}/route_segments.cc
	# ${PNC_MAP_SRCS_ROOT}/pnc_map.cc

	${COMMON_SRCS_ROOT}/configs/config_gflags.cc
	# modules/planning/common/planning_gflags.cc
	# modules/routing/common/routing_gflags.cc
)

target_link_libraries(hdmap
	absl::strings
	# ${ABSL_PUBLIC_LIBRARIES}
	${Boost_LIBRARIES}
	# pnc_cuda_util
	tinyxml2
	proj
	${Protobuf_LIBRARIES}
	${absl_LIBS}
    
	glog
	gflags
)

add_executable(test_api
    example/test_api.cc
)

target_link_libraries(test_api
	${Boost_LIBRARIES}
	hdmap
	tinyxml2
	proj
	absl::strings

	${Protobuf_LIBRARIES}
	glog
	gflags
)

add_executable(test_map
    example/test_map.cc
)

target_link_libraries(test_map
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	hdmap
	tinyxml2
	proj
	absl::strings

	${Protobuf_LIBRARIES}
	glog
	gflags
)

add_executable(test_path
    example/test_path.cc
)

target_link_libraries(test_path
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	hdmap
	tinyxml2
	proj
	absl::strings

	${Protobuf_LIBRARIES}
	glog
	gflags
)

install(TARGETS hdmap
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# EXECUTE_PROCESS(
# 	COMMAND ../headers.sh
# )

install(DIRECTORY include 
	DESTINATION .
	PATTERN "*.h"
    PERMISSIONS OWNER_READ GROUP_READ
)

# install(TARGETS hdmap 
# 	RUNTIME DESTINATION bin
# 	LIBRARY DESTINATION lib
# 	ARCHIVE DESTINATION lib
# )
