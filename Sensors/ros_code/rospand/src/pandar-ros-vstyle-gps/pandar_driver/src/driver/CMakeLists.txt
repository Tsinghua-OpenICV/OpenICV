# build the driver node
add_executable(pandar_node pandar_node.cc driver.cc)
add_dependencies(pandar_node pandar_driver_gencfg)
target_link_libraries(pandar_node
  pandar_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# build the nodelet version
add_library(driver_nodelet nodelet.cc driver.cc)
add_dependencies(driver_nodelet pandar_driver_gencfg)
target_link_libraries(driver_nodelet
  pandar_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# install runtime files
install(TARGETS pandar_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)
install(TARGETS driver_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
