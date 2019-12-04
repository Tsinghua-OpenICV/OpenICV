if(OPENICV_WITH_OPENCV)
    add_subdirectory(OpenCV)
    icv_set_solution_folder(icvExtensionOpenCV Extensions)
endif()

if(OPENICV_WITH_PCL)
    add_subdirectory(PCL)
    icv_set_solution_folder(icvExtensionPCL Extensions)
endif()

if(OPENICV_WITH_EIGEN)
    add_subdirectory(Eigen)
    icv_set_solution_folder(icvExtensionEigen Extensions)
endif()
