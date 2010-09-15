add_library(sr
	SenderBase.cc
	SimpleSender.cc
	ThreadedSender.cc
	sr_ecp.cc
	sr_edp.cc
	sr_ui.cc
	sr_vsp.cc
	srlib.cc
)

if(USE_MESSIP)
target_link_libraries(sr messip)
endif(USE_MESSIP)
