add_library(ecp_generators_force 
	ecp_g_weight_measure.cc
)

target_link_libraries(ecp_generators_force ecp_mp_generators ecp_mp)

install(TARGETS ecp_generators_force DESTINATION lib)
