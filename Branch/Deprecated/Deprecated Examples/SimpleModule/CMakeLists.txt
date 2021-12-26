add_library(SimpleModule SHARED TimesFunction.cxx)
# TODO: How to reduce the module library size?
target_link_libraries(SimpleModule icvCore)

add_custom_command(TARGET SimpleModule PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different SimpleModule.json $<TARGET_FILE_DIR:SimpleModule>
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    COMMENT "Copy configuration files")
