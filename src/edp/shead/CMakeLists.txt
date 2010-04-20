add_executable(edp_shead
	edp_e_shead.cc
)

target_link_libraries(edp_shead kinematicsshead kinematics edp mrrocpp
	${COMMON_LIBS})

install(TARGETS edp_shead RUNTIME DESTINATION bin)

