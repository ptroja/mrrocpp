add_library(ecp_subtasks
	ecp_st_smooth_file_from_mp.cc
	ecp_st_bias_edp_force.cc
	ecp_st_tff_nose_run.cc
)

target_link_libraries(ecp_subtasks ecp)

