add_executable(edp_smb
	edp_e_smb.cc
	../common/int_hand_suplement.cc
)

target_link_libraries(edp_smb kinematicssmb edp
	${COMMON_LIBRARIES})

install(TARGETS edp_smb DESTINATION bin)

