


FIND_PACKAGE( PCL 1.2 REQUIRED )
FIND_PACKAGE( OpenCV  REQUIRED )
FIND_PACKAGE( Eigen3  REQUIRED )

# TODO: How to reduce the module library size?
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${PCL_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include )


add_library(icvSlamLidarRT SHARED SlamLidarRT.cxx )

TARGET_LINK_LIBRARIES( icvSlamLidarRT icvCore ${OpenCV_LIBS}  ${PCL_LIBRARIES ${Boost_LIBRARIES}}

)

install(TARGETS icvSlamLidarRT
    COMPONENT icvSlamLidarRT
    RUNTIME DESTINATION bin
)
