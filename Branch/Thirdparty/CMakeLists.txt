cmake_minimum_required(VERSION 3.4)
set(THIRDPARTY_PATH ${CMAKE_CURRENT_SOURCE_DIR})

if(OPENICV_R_BOOST)
add_subdirectory(src/boost)
endif()

