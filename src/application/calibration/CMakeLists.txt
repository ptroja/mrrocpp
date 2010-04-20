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
        ${ECP_MPDIR}/sensor/ecp_mp_s_cvfradia.cc
)

add_executable(ecp_t_axzb_force
	ecp_t_calib_axzb.cc
	ecp_t_calibration.cc
	ecp_st_acq_force.cc
	ecp_st_acquisition.cc
	ecp_t_axzb_force.cc
	${ECPDIR}/common/generator/ecp_g_smooth.cc
	ecp_g_pcbird_nose_run.cc
	${ECPDIR}/common/generator/ecp_g_force.cc
	${ECP_MPDIR}/birdclient.cc
	${ECP_MPDIR}/sensor/ecp_mp_s_pcbird.cc
)

add_executable(ecp_t_kcz_test
	ecp_t_kcz_test.cc
	${ECPDIR}/common/generator/ecp_g_smooth.cc
	${ECP_MPDIR}/birdclient.cc
	${ECP_MPDIR}/sensor/ecp_mp_s_pcbird.cc
)

SET(EXECUTABLES ecp_t_axzb_eih ecp_t_axzb_force ecp_t_kcz_test)
FOREACH(executable ${EXECUTABLES})
	target_link_libraries(${executable}
		ecp_r_irp6ot ecp_r_irp6p
		kinematicsirp6ot kinematicsirp6p kinematics
		ecp ecp_mp
		${COMMON_LIBS}
		${GSL_LIBRARIES}
	)
ENDFOREACH(executable)
install(TARGETS ${EXECUTABLES} RUNTIME DESTINATION bin)
