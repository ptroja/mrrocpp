# find the GSL
if(NOT QNXNTO)
	set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
	find_package(GSL)
	include_directories (${GSL_INCLUDE_DIR})
else(NOT QNXNTO)
	find_library(GSL_LIBRARY gsl $ENV{QNX_TARGET}/mrlib/lib)
	find_library(GSLCBLAS_LIBRARY gslcblas $ENV{QNX_TARGET}/mrlib/lib)
	set(GSL_LIBRARIES ${GSL_LIBRARY} ${GSLCBLAS_LIBRARY})
	# TODO: find a GSL headers
endif(NOT QNXNTO)

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

add_executable(ecp_t_axxb_eih
	ecp_t_calib_axxb.cc
	ecp_t_calibration.cc
	ecp_st_acq_eih.cc
	ecp_st_acquisition.cc
	ecp_t_axxb_eih.cc
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
	ecp_g_pcbird_nose_run.cc
	${ECPDIR}/common/generator/ecp_g_force.cc
	${ECP_MPDIR}/birdclient.cc
	${ECP_MPDIR}/sensor/ecp_mp_s_pcbird.cc
)

add_executable(ecp_t_kcz_test
	ecp_t_kcz_test.cc
	${ECP_MPDIR}/birdclient.cc
	${ECP_MPDIR}/sensor/ecp_mp_s_pcbird.cc
)

SET(EXECUTABLES ecp_t_axzb_eih ecp_t_axxb_eih ecp_t_axzb_force ecp_t_kcz_test)
FOREACH(executable ${EXECUTABLES})
	target_link_libraries(${executable}
		ecp_r_irp6ot_m ecp_r_irp6p_m
		ecp
		${GSL_LIBRARIES}
	)
ENDFOREACH(executable)
install(TARGETS ${EXECUTABLES} DESTINATION bin)
