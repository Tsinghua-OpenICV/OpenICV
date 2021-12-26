#execute_process(COMMAND sudo apt-get install libmsgpack-dev)
#@Deprecated

string(APPEND msg_link "http://msgpack.org/releases/cpp/msgpack-0.5.7.tar.gz")
string(APPEND MSGdown ${OpenICV_ini_path} "/Third_build" )

string(APPEND TARFILENAME_msg "msgpack-0.5.7.tar.gz" )
string(APPEND msgbuild ${MSGdown} "/msgpack-0.5.7" )

message(${msgbuild})
execute_process(COMMAND mkdir ${MSGdown})
execute_process(COMMAND wget ${msg_link}
		WORKING_DIRECTORY ${MSGdown})

execute_process(COMMAND	tar zxvf ${TARFILENAME_msg}

		WORKING_DIRECTORY ${MSGdown})

execute_process(COMMAND sh ./configure
		WORKING_DIRECTORY ${msgbuild})
execute_process(COMMAND  sudo make install 
		WORKING_DIRECTORY ${msgbuild})

