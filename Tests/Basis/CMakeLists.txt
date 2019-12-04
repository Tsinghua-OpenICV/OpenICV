icv_set_module_names(BasisTest)
file(GLOB_RECURSE ${MODULE_SRC_NAME} *test.cxx)

# depends on header only boost libraries: array

configure_file(../icvTest.cxx.in icvTest.cxx)
list(APPEND ${MODULE_SRC_NAME} icvTest.cxx)

add_executable(${MODULE_TARGET_NAME} ${${MODULE_SRC_NAME}})
target_link_libraries(${MODULE_TARGET_NAME} PUBLIC icvBasis ${Boost_LIBRARIES})
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})

add_test(NAME ${MODULE_TARGET_NAME}
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
         COMMAND ${MODULE_TARGET_NAME})
