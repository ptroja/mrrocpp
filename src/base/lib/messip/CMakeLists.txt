add_library(messip
	logg_messip.c messip_lib.c
	messip_utils.c messip_dataport.cc)

target_link_libraries (messip ${COMPATIBILITY_LIBRARIES}) 

install(TARGETS messip DESTINATION lib)



add_executable(messip_mgr messip_mgr.c)
target_link_libraries (messip_mgr messip
	${CMAKE_THREAD_LIBS_INIT} ${COMPATIBILITY_LIBRARIES} pthread) 
install(TARGETS messip_mgr DESTINATION bin)

add_executable(messip_sin messip_sin.c)
target_link_libraries (messip_sin messip
	${CMAKE_THREAD_LIBS_INIT} ${COMPATIBILITY_LIBRARIES} pthread)
install(TARGETS messip_sin DESTINATION bin)


#add_executable(server server.c)
#target_link_libraries (server messip ${COMPATIBILITY_LIBRARIES})
#add_executable(client client.c)
#target_link_libraries (client messip ${COMPATIBILITY_LIBRARIES})
