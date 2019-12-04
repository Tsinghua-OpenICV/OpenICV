icv_set_module_vars(Gps)

file(GLOB_RECURSE ${MODULE_HDR_NAME} *.h)
file(GLOB_RECURSE MODULE_CPPG_NAME "*.CPP")
file(GLOB_RECURSE MODULE_CPP_NAME "*.cpp")
find_package(PCL 1.2 REQUIRED ) 
add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}}   ${MODULE_CPP_NAME} ${MODULE_CPPG_NAME} )

message("header files =  ${MODULE_HD_NAME} ")
# TODO: How to reduce the module library size?


#message("cpp= ${MODULE_CPP_NAME}")
target_link_libraries(${MODULE_TARGET_NAME} icvCore ${Boost_LIBRARIES} )
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS} )

icv_set_solution_folder(${MODULE_TARGET_NAME} Sensors)

icv_install_module(Gps)
