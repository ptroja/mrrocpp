add_executable(edp_smb
	edp_e_smb.cc
)

target_link_libraries(edp_smb kinematicssmb kinematics edp mrrocpp
	${COMMON_LIBS})

install(TARGETS edp_smb RUNTIME DESTINATION bin)

