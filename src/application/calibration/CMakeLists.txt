# find the GSL

	set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
	find_package(GSL)
	include_directories (${GSL_INCLUDE_DIR})


add_executable(ecp_t_axzb_eih
	ecp_t_calib_axzb.cc
	ecp_t_calibration.cc
	ecp_st_acq_eih.cc
	ecp_st_acquisition.cc
	ecp_t_axzb_eih.cc
	ecp_g_eihcalibration.cc
	ecp_g_eih_nose_run.cc
)

add_executable(ecp_t_axxb_eih
	ecp_t_calib_axxb.cc
	ecp_t_calibration.cc
	ecp_st_acq_eih.cc
	ecp_st_acquisition.cc
	ecp_t_axxb_eih.cc
	ecp_g_eihcalibration.cc
	ecp_g_eih_nose_run.cc
)

add_executable(ecp_t_axzb_force
	ecp_t_calib_axzb.cc
	ecp_t_calibration.cc
	ecp_st_acq_force.cc
	ecp_st_acquisition.cc
	ecp_t_axzb_force.cc
	ecp_g_pcbird_nose_run.cc
	../../sensor/pcbird/birdclient.cc
	../../sensor/pcbird/ecp_mp_s_pcbird.cc
)

add_executable(ecp_t_kcz_test
	ecp_t_kcz_test.cc
	../../sensor/pcbird/birdclient.cc
	../../sensor/pcbird/ecp_mp_s_pcbird.cc
)

SET(EXECUTABLES ecp_t_axzb_eih ecp_t_axxb_eih ecp_t_axzb_force ecp_t_kcz_test)
FOREACH(executable ${EXECUTABLES})
	target_link_libraries(${executable}
		ecp_r_irp6ot_m ecp_r_irp6p_m
		ecp_generators
		${GSL_LIBRARIES}
	)
ENDFOREACH(executable)
install(TARGETS ${EXECUTABLES} DESTINATION bin)
