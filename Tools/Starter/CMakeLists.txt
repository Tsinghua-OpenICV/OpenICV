find_package(Boost COMPONENTS  program_options  system filesystem log REQUIRED)

add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions(-DBOOST_LOG_DYN_LINK)
add_executable(icvStarter Entry.cxx)
if(UNIX)
	set(Opt_lib pthread)
else()
	set(Opt_lib )
endif()
target_link_libraries(icvStarter PUBLIC icvEngine icvBasis  ${Opt_lib} ${Boost_LIBRARIES})
target_include_directories(icvStarter SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})

install(TARGETS icvStarter
    COMPONENT Tools
    RUNTIME DESTINATION bin
)
