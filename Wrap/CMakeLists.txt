option(OPENICV_ENABLE_PYTHON "Enable python function support" OFF)
option(OPENICV_ENABLE_CSHARP "Enable csharp function support" OFF)

if(OPENICV_ENABLE_PYTHON)
    add_subdirectory(Python)
endif()
