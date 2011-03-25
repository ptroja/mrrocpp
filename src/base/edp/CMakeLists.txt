add_library(edp
	edp_m.cc edp_effector.cc edp_shell.cc
	edp_e_manip.cc edp_e_motor_driven.cc
	edp_force_sensor.cc
	servo_gr.cc regulator.cc in_out.cc
	trans_t.cc manip_trans_t.cc vis_server.cc reader.cc
)

install(TARGETS edp DESTINATION lib)
