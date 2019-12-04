if(OPENICV_BUILD_TESTS)
    find_package(Boost REQUIRED unit_test_framework)
    add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)

    add_subdirectory(Core)
    add_subdirectory(Basis)
    add_subdirectory(Engine)
    add_subdirectory(Extensions)

    icv_set_solution_folder(icvCoreTest Tests)
    icv_set_solution_folder(icvBasisTest Tests)
    icv_set_solution_folder(icvEngineTest Tests)
    
    if(OPENICV_ENABLE_REMOTE)
        add_subdirectory(Net)
        icv_set_solution_folder(icvNetTest Tests)
    endif()
endif()
