add_executable(edp_spkm
	edp_e_spkm.cc
)

target_link_libraries(edp_spkm kinematicsspkm kinematics edp mrrocpp
	${COMMON_LIBS})

install(TARGETS edp_spkm RUNTIME DESTINATION bin)

