/*!
 * @file mp_t_swarmitfix_demo_agent1_warsaw.cpp
 *
 * @date Dec 29, 2011
 * @author tkornuta
 */

#include "mp_t_swarmitfix_demo_agent1_warsaw.h"

namespace mrrocpp {
namespace mp {
namespace task {

#define SMB_WALK 0
#define SMB_PULL_LEGS 0

task* return_created_mp_task(lib::configurator &_config)
{
	return new swarmitfix_demo_agent1_warsaw(_config);
}


swarmitfix_demo_agent1_warsaw::swarmitfix_demo_agent1_warsaw(lib::configurator &config_) :
		swarmitfix_demo_base(config_)
{

}

void swarmitfix_demo_agent1_warsaw::create_robots()
{
	// Activate robots (depending on the configuration settings).
	ACTIVATE_MP_ROBOT(spkm1);
	ACTIVATE_MP_ROBOT(smb1);
	ACTIVATE_MP_ROBOT(shead1);
}


void swarmitfix_demo_agent1_warsaw::main_task_algorithm(void)
{
	/*
	 neutral pose OK
	 tool:  -0.1412 -0.04 0.5718 3.1416 0.137 0
	 wrist: 0.15 -0.04 0.4 0 -0.92 0
	 */
	// Move to the *neutral* PKM pose.
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, 0.15, -0.04, 0.4, 0, -0.92, 0);
	// Move shead to synchro position.
	move_shead_joints(0.0);

#if(SMB_PULL_LEGS)
	// Pull out all SMB legs.
	move_smb_legs(lib::smb::OUT, lib::smb::OUT, lib::smb::OUT);
#endif
	/*
	 Podparcie nr 1: smb rot = 1
	 tool:  -0.33 0.0012 0.625 -0.8193 0.029 -2.3437
	 wrist: -0.0693 0 0.4097 0 -0.763 -0.03

	 odejście:
	 tool:  -0.33 0 0.59 3.1416 0.02 0
	 wrist: -0.0609 0 0.3853 0 -0.803 0
	 */
	move_to_pose_and_return(-0.0693, 0, 0.4097, 0, -0.763, -0.03, -0.0609, 0, 0.3853, 0, -0.803, 0, 1, 0.5);
	/*
	 Podparcie 2: smb rot = 0, legs out
	 tool:   -0.2919 0 0.626 -3.1416 -0.03 0
	 wrist:  -0.0333 0 0.4081 0 -0.753 0

	 odejście z podparcia nr 1:
	 tool:  -0.2919 0 0.6 -3.1416 0 0
	 wrist: -0.0269 0 0.39 0 -0.783 0
	 */
	move_to_pose_and_return(-0.0333, 0, 0.4081, 0, -0.753, 0, -0.0269, 0, 0.39, 0, -0.783, 0, 0, -0.5);
	/*
	 Podparcie nr 3: smb rot = -1
	 tool: -0.3691 0 0.627 3.1416 -0.05 0
	 wrist: -0.1149 0 0.404 0 -0.733 0

	 odejście:
	 tool:  -0.3691 0 0.5847 3.1416 0.02 0
	 wrist: -0.1 0 0.38 0 -0.803 0
	 */
	move_to_pose_and_return(-0.1149, 0, 0.404, 0, -0.733, 0, -0.1, 0, 0.38, 0, -0.803, 0, -1, 0);

#if(SMB_WALK)
	// Move to SMB position 2 - rotate around leg 3 by 60 degrees.
	rotate_smb(3, 1);

	// Move SMB and SPKM to pose 4.
	move_smb_external(0, 2.0);
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, -0.05, 0, 0.425, 0, -0.81, -0.1);
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, -0.05, 0, 0.445, 0, -0.81, -0.1);
	wait_ms(1000);

	// Move back to the *neutral* PKM pose.
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, -0.05, 0, 0.425, 0, -0.81, -0.1);
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, 0.15, 0, 0.405, 0, -1.045, 0);

	// Move SMB and SPKM to pose 3.
	move_smb_external(0, -1.500);
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, -0.1, 0, 0.409, 0, -0.868, 0.09);
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, -0.1, 0, 0.429, 0, -0.868, 0.09);
	wait_ms(1000);

	// Move back to the *neutral* PKM pose.
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, -0.1, 0, 0.419, 0, -0.868, 0.09);
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, 0.15, 0, 0.405, 0, -1.045, 0);

	// Move SMB and SPKM to pose 5.
	move_smb_external(0, -3.0);
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, -0.1, 0, 0.404, 0, -0.83, -0.02);
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, -0.1, 0, 0.424, 0, -0.83, -0.02);
	wait_ms(1000);

	// Move back to the *neutral* PKM pose.
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, -0.1, 0, 0.404, 0, -0.83, -0.02);
	move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, 0.15, 0, 0.405, 0, -1.045, 0);

	// Move to SMB position 1 - rotate around leg 3 by -60 degrees.
	rotate_smb(3, -1);
	// Move to start position.
	move_smb_external(0, 0);
#endif

	// Rotate SMB to synchro position.
	move_smb_external(0.0, 0.0);
	wait_ms(1000);

#if(SMB_PULL_LEGS)
	move_smb_legs(lib::smb::IN, lib::smb::IN, lib::smb::IN);
#endif
// KONIEC
//	sr_ecp_msg->message("END");

}


} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */
