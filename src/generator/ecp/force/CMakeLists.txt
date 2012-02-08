add_library(ecp_generators_force 
	ecp_g_weight_measure.cc
	ecp_g_tff_nose_run.cc
	ecp_g_tff_rubik_face_rotate.cc
)

add_library(ecp_mp_generators_force
	ecp_mp_g_tff_nose_run.cc
	)

target_link_libraries(ecp_generators_force ecp_mp_generators ecp_mp)

install(TARGETS ecp_generators_force ecp_mp_generators_force DESTINATION lib)
