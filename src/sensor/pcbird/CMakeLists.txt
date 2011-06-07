add_library(pcbird_sensor
	birdclient.cc
	ecp_mp_s_pcbird.cc
)

if(QNXNTO)
target_link_libraries (pcbird_sensor socket)
endif(QNXNTO)

install(TARGETS pcbird_sensor DESTINATION lib)