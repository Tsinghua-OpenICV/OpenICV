add_executable(pandar_cloud_node cloud_node.cc convert.cc driver.cc)
add_dependencies(pandar_cloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pandar_cloud_node pandar_rawdata
					  pandar_input
					  pcap
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_library(pandar_cloud_nodelet cloud_nodelet.cc convert.cc driver.cc)
add_dependencies(pandar_cloud_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pandar_cloud_nodelet 
					  pandar_rawdata 
					  pandar_input
					  pcap
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(pandar_ringcolors_node ringcolors_node.cc colors.cc)
target_link_libraries(pandar_ringcolors_node
					  ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_library(pandar_ringcolors_nodelet ringcolors_nodelet.cc colors.cc)
target_link_libraries(pandar_ringcolors_nodelet
					  ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(pandar_transform_node transform_node.cc transform.cc)
add_dependencies(pandar_transform_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pandar_transform_node pandar_rawdata
					  ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_library(pandar_transform_nodelet transform_nodelet.cc transform.cc)
add_dependencies(pandar_transform_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pandar_transform_nodelet pandar_rawdata
					  ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS 
            pandar_cloud_node
            pandar_cloud_nodelet
            pandar_ringcolors_node
            pandar_ringcolors_nodelet
            pandar_transform_node
            pandar_transform_nodelet
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
