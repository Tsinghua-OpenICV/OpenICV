icv_set_module_names(ExtensionPCLTest)
file(GLOB_RECURSE ${MODULE_SRC_NAME} *test.cxx)

configure_file(../../icvTest.cxx.in icvTest.cxx)
list(APPEND ${MODULE_SRC_NAME} icvTest.cxx)


add_executable(${MODULE_TARGET_NAME} ${${MODULE_SRC_NAME}})

# XXX: icvExtensionPCL is currently header only, depends on icvBasis instead
set(Boost_Test_LIBRARIES ${Boost_LIBRARIES}) # prevent being overwriitten by find_package(PCL REQUIRED)
find_package(PCL REQUIRED)
target_link_libraries(${MODULE_TARGET_NAME} PUBLIC icvBasis ${PCL_LIBRARIES} ${Boost_Test_LIBRARIES})

target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})

add_test(NAME ${MODULE_TARGET_NAME}
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
         COMMAND ${MODULE_TARGET_NAME})
