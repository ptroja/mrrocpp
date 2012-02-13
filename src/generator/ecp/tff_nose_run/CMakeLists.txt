add_library(ecp_g_tff_nose_run
	ecp_g_tff_nose_run.cc
)

add_library(ecp_mp_g_tff_nose_run
	ecp_mp_g_tff_nose_run.cc
	)
	
target_link_libraries(ecp_g_tff_nose_run ecp)

install(TARGETS ecp_g_tff_nose_run ecp_mp_g_tff_nose_run DESTINATION lib)
