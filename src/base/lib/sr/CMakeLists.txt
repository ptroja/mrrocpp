add_library(sr
	Sender.cc
	sr_ecp.cc
	sr_edp.cc
	sr_ui.cc
	sr_vsp.cc
	srlib.cc
)

target_link_libraries(sr messip)

install(TARGETS sr DESTINATION lib)
