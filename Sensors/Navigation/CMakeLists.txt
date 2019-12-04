icv_set_module_vars(Navigation)
find_package(Qt5Widgets REQUIRED) 

add_definitions(

${QT_DEFINITIONS})
include_directories(
${Qt5Widgets_INCLUDE_DIRS}

)

file(GLOB_RECURSE MODULE_HDR_NAME *.h)

file(GLOB_RECURSE MODULE_CPP_NAME "*.cpp")

message("current==${MODULE_HDR_NAME}")


add_library(icvNavigation SHARED demo.cxx ${MODULE_HDR_NAME}   ${MODULE_CPP_NAME}
 )
# TODO: How to reduce the module library size?


target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS} )
target_link_libraries(icvNavigation 
icvCore  
${Boost_LIBRARIES} 
Qt5::Core 
Qt5::Gui 
Qt5::Widgets 
)

icv_set_solution_folder(icvNavigation Sensors)

icv_install_module(Navigation)
