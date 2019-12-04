# Find python
find_package(PythonInterp 3.0 REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)
find_package(NumPy REQUIRED)

# Find boost.python
# set(version ${PYTHONLIBS_VERSION_STRING})

# STRING( REGEX REPLACE "[^0-9]" "" boost_py_version ${version} )
# find_package(Boost REQUIRED "python-py${boost_py_version}")
# set(Boost_PYTHON_FOUND ${Boost_PYTHON-PY${boost_py_version}_FOUND})

# while(NOT "${version}" STREQUAL "" AND NOT Boost_PYTHON_FOUND)
#     STRING( REGEX REPLACE "([0-9.]+).[0-9]+" "\\1" version ${version})
#     STRING( REGEX REPLACE "[^0-9]" "" boost_py_version ${version})
#     find_package(Boost REQUIRED "python-py${boost_py_version}")
#     set(Boost_PYTHON_FOUND ${Boost_PYTHON-PY${boost_py_version}_FOUND})

#     STRING( REGEX MATCHALL "([0-9.]+).[0-9]+" has_more_version ${version} )
#     if("${has_more_version}" STREQUAL "")
#         break()
#     endif()
# endwhile()
# if(NOT Boost_PYTHON_FOUND)
#     find_package(Boost REQUIRED python)
# endif()

add_library(pyicvCore SHARED _icvCore.cxx)
icv_set_solution_folder(pyicvCore Python)
set_target_properties(pyicvCore PROPERTIES PREFIX "" OUTPUT_NAME "_icvCore")
target_link_libraries(pyicvCore PUBLIC icvCore ${PYTHON_LIBRARIES} ${Boost_LIBRARIES})
target_include_directories(pyicvCore SYSTEM PUBLIC ${PYTHON_INCLUDE_DIRS} ${NUMPY_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
