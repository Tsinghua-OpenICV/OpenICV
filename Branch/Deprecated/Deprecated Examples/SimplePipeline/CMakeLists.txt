add_executable(SimplePipeline
    Entry.cxx AddFunction.hxx PolynomialFunction.hxx)
target_link_libraries(SimplePipeline icvEngine icvBasis)

add_custom_command(TARGET SimplePipeline PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different SimplePipeline.json $<TARGET_FILE_DIR:SimplePipeline>
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    COMMENT "Copy configuration files")
