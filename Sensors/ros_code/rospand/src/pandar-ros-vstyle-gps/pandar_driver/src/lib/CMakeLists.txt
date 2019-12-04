add_library(pandar_input input.cc)
target_link_libraries(pandar_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(pandar_input ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS pandar_input
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
