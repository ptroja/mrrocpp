if(NOT UBUNTU32BIT)
add_library(ecp_subtasks
	ecp_st_smooth_file_from_mp.cc
	ecp_st_tff_nose_run.cc
	)

add_library(ecp_mp_subtasks
	ecp_mp_st_tff_nose_run.cc
	)

target_link_libraries(ecp_subtasks ecp_mp_subtasks ecp_generators)
install(TARGETS ecp_subtasks ecp_mp_subtasks DESTINATION lib)
endif(NOT UBUNTU32BIT)