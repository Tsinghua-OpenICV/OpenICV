if(OPENICV_WITH_OPENCV)
    add_subdirectory(OpenCV)
    icv_set_solution_folder(icvExtensionOpenCVTest Tests)
endif()

if(OPENICV_WITH_PCL)
    add_subdirectory(PCL)
    icv_set_solution_folder(icvExtensionPCLTest Tests)
endif()

if(OPENICV_WITH_EIGEN)
    add_subdirectory(Eigen)
    icv_set_solution_folder(icvExtensionEigenTest Tests)
endif()
