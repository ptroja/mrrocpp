include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(ui
	../MAM_wnd_manual_moves_automatic_measure.cc
	../abmain.cc
	../fun.cc
	../fun_r_conveyor.cc
	../fun_r_irp6_mechatronika.cc
	../fun_r_irp6_on_track.cc
	../fun_r_irp6_postument.cc
	../fun_r_irp6ot_tfg.cc
	../fun_r_irp6p_tfg.cc
	#../fun_r_polycrank.cc
	../fun_r_shead.cc
	../fun_r_smb.cc
	../fun_r_speaker.cc
	../fun_r_spkm.cc
	../threads.cc
	../timer.cc
	../ui.cc
	../ui_ecp_r_common.cc
	../ui_ecp_r_irp6_common.cc
	../ui_ecp_r_speaker.cc
	../ui_ecp_r_tfg_and_conv.cc
	../ui_init.cc
	../wndFileLocationEvents.cc
	../wndForceControlEvents.cc
	../wndTrajectoryReproduceEvents.cc
)

find_library(PHEX_LIBRARY phexlib $ENV{QNX_TARGET}/x86/usr/lib)
find_library(AP_LIBRARY Ap $ENV{QNX_TARGET}/x86/usr/lib)
find_library(PH_LIBRARY ph $ENV{QNX_TARGET}/x86/usr/lib)
set(PHOTON_LIBRARIES ${PHEX_LIBRARY} ${AP_LIBRARY} ${PH_LIBRARY})

set(ROBOT_LIBRARIES
	ecp_r_irp6ot ecp_r_irp6p
	ecp_r_irp6ot_tfg ecp_r_irp6p_tfg
	ecp_r_irp6m ecp_r_conv
	ecp_r_polycrank
	ecp_r_speaker
	ecp_r_smb ecp_r_spkm ecp_r_shead
)

set(KINEMATICS_LIBRARIES
	kinematicsirp6ot kinematicsirp6p
	kinematicsirp6ot_tfg kinematicsirp6p_tfg
	kinematicsspkm kinematicssmb kinematicsshead
)

target_link_libraries(ui
	${ROBOT_LIBRARIES}
	${KINEMATICS_LIBRARIES}
	ecp
	${PHOTON_LIBRARIES}
	${COMMON_LIBRARIES}
)

install(TARGETS ui DESTINATION bin)
