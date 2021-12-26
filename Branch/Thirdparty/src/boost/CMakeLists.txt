#string(APPEND boost_link "https://dl.bintray.com/boostorg/release/1.69.0/source/boost_1_69_0.tar.gz" )
string(APPEND boost_link "https://sourceforge.net/projects/boost/files/boost/1.66.0/boost_1_66_0.tar.gz")
string(APPEND boostdown ${OpenICV_ini_path} "/boost_build" )
string(APPEND boostinstall ${BOOST_ROOT} )
string(APPEND TARFILENAME "boost_1_66_0.tar.gz" )
string(APPEND boostbuild ${boostdown} "/boost_1_66_0" )

message(${boostdown})
execute_process(COMMAND mkdir ${boostdown}
		COMMAND wget ${boost_link}
		WORKING_DIRECTORY ${boostdown})

execute_process(COMMAND	tar zxvf ${TARFILENAME}

		WORKING_DIRECTORY ${boostdown})

execute_process(COMMAND sh ./bootstrap.sh --with-libraries=program_options,thread,system,program_options,log
		WORKING_DIRECTORY ${boostbuild})
execute_process(COMMAND  ./b2
		COMMAND  ./b2 install --prefix=${boostinstall}
		WORKING_DIRECTORY ${boostbuild})




message("")
message("")
message("")
message("")
#find_package(Boost REQUIRED thread)
# TODO: wrap boost and build boost statically
