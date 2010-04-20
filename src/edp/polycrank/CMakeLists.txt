add_executable(edp_polycrank
	edp_e_polycrank.cc
)

target_link_libraries(edp_polycrank kinematicspolycrank kinematics edp mrrocpp
	${COMMON_LIBS})

install(TARGETS edp_polycrank RUNTIME DESTINATION bin)

