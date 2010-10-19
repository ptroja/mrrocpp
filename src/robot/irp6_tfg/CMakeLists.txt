add_library(kinematic_model_irp6_tfg
kinematic_model_irp6_tfg.cc
)



add_executable(ecp_tfg
	ecp_g_tfg.cc
	ecp_t_tfg.cc
)

target_link_libraries(ecp_tfg
	ecp_r_irp6p_tfg ecp_r_irp6ot_tfg
	ecp
)

install(TARGETS ecp_tfg DESTINATION bin)
