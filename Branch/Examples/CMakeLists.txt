if(OPENICV_WITH_OPENCV)
    add_subdirectory(OpenCV)
    icv_set_solution_folder(icvExtensionOpenCV Extensions)
endif()

add_subdirectory(UsefulFunction)

add_subdirectory(HelloPython)
