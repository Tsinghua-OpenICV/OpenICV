project (ins_lib)

cmake_minimum_required (VERSION 2.8.4) # This version was released 2011-02-16

include_directories(
  ./
)
add_library(lib_ins 
                          KFApp.cpp
                          PSINS.cpp)